use std::{collections::HashMap, sync::mpsc::Sender};

use crate::{
  srcp_devices_ddl::SRCPDeviceDDL,
  srcp_protocol_ddl::{DdlProtokolle, HashMapProtokollVersion},
  srcp_server_types::{SRCPMessage, SRCPMessageID, SRCPMessageType},
};

/// SM Device
pub struct DdlSM {
  //SRCP Bus auf dem gearbeitet wird
  _bus: usize, //Wird hier nicht verwendet, da keine SCRP Info Message gesendet werden.
  //Sender für SRCP Antworten
  tx: Sender<SRCPMessage>,
  //Alle vorhandenen Protokollimplementierungen mit allen Versionen
  all_protokolle: HashMapProtokollVersion,
  //Konvertierung auf bei GL/GA verwendete (Protokollnamen, Version)
  gl_ga_prot_names: HashMap<String, (String, String)>,
  //Aktuell verwendetes SM Protokoll und Version, durch INIT gesetzt.
  sm_protokoll: Option<(DdlProtokolle, String)>,
}

impl DdlSM {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * bus - SRCP Bus auf dem dieses Device arbeitet
  /// * tx - Sender für Info Messages / Antworten an SRCP Clients
  /// * all_protokolle - Alle vorhandenen Protokollimplementierungen mit allen Versionen
  pub fn new(
    bus: usize, tx: Sender<SRCPMessage>, all_protokolle: HashMapProtokollVersion,
  ) -> DdlSM {
    let mut gl_ga_prot_names: HashMap<String, (String, String)> = HashMap::new();
    gl_ga_prot_names.insert(
      "NMRA".to_string(),
      (DdlProtokolle::Dcc.to_string(), "2".to_string()),
    );
    gl_ga_prot_names.insert(
      "MFX".to_string(),
      (DdlProtokolle::Mfx.to_string(), "0".to_string()),
    );
    DdlSM {
      _bus: bus,
      tx,
      all_protokolle,
      gl_ga_prot_names,
      sm_protokoll: None,
    }
  }
}

impl SRCPDeviceDDL for DdlSM {
  /// Empfangenes Kommando validieren.
  /// Return true wenn Kommando Ok.
  /// Sendet die Antwort Message (Ok / Err) an Sender zurück.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn validate_cmd(&self, cmd_msg: &SRCPMessage) -> bool {
    let mut result = false;
    //Für GA wird unterstützt: INIT, TERM, SET, GET
    if let SRCPMessageID::Command { msg_type } = cmd_msg.message_id {
      match msg_type {
        SRCPMessageType::INIT => {
          //Format ist INIT <bus> SM <protocol>
          //Für <protocol> wird aktuell "NMRA" und "MFX" unterstützt
          //Leider ist in der srcp Protkollspez. hier "NMRA" definiert, bei GL/GA ist "N" definiert :-(
          //Um alles gleich machen zu können übersetzen wir hier zurück auf die Definitionen bei GL/GA
          if cmd_msg.parameter.len() >= 1 {
            //Das verlangte Protokoll muss hier existieren
            if self.gl_ga_prot_names.contains_key(&cmd_msg.parameter[0]) {
              //Und aktuell keines in Verwendung sein
              if self.sm_protokoll.is_none() {
                result = true;
              } else {
                self
                  .tx
                  .send(SRCPMessage::new_err(cmd_msg, "415", "forbidden"))
                  .unwrap();
              }
            } else {
              self
                .tx
                .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
                .unwrap();
            }
          } else {
            self
              .tx
              .send(SRCPMessage::new_err(
                cmd_msg,
                "420",
                "unsupported device protocol",
              ))
              .unwrap();
          }
        }
        SRCPMessageType::TERM => {
          //Format ist TERM <bus> SM [<protocol>]
          //Optionale <protocol> Angabe aufgrund Kompatbilität altes srcpd bei dem ich für MFX hier "MFX" definiert habe.
          //Ist aber gar nicht notwendig, es wird einfach immer TERM für alle Protokolle gemacht.
          //TERM ist gültig, wenn zuvor ein anderes SM Protokoll mit INIT aktiviert wurde
          if self.sm_protokoll.is_some() {
            result = true;
          } else {
            self
              .tx
              .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
              .unwrap();
          }
        }
        SRCPMessageType::SET | SRCPMessageType::GET => {
          //Format ist SET <bus> SM <decoderaddress> <type> <values ...> <set value>
          //<type> ist Protokollabhängig (z.B. bei NMRA CV, CVBIT, REG, PAGE, bei MFX CAMFX)
          //Anzahl weitere Parameter ist auch Protokollabhängig (z.B. NMRA CV: CV, Value, bei MFX CAMFX Block, CA, CAIndex, Index, Value)
          //<set value> nur bei SET, nicht bei GET
          //Es muss ein Protokoll mit INIT für SM ausgewählt worden sein
          if let Some((prot, prot_ver)) = &self.sm_protokoll {
            if cmd_msg.parameter.len() > 2 {
              //Type prüfen. Protokoll muss "sm_get_all_types" != None liefern, sonst hätte es mit "INIT" nicht aktiviert werden können
              let protokoll = &self.all_protokolle[prot][prot_ver.as_str()];
              if let Some(para_count) = protokoll
                .borrow()
                .sm_get_all_types()
                .unwrap()
                .get(&cmd_msg.parameter[1])
              {
                //Protokoll ist initalisiert, für Protokoll gültiger Type ist angegeben
                //Prüfung notwendige Anzahl Parameter
                if cmd_msg.parameter.len()
                  == (2 //2+ ist Dekoderadr und Type
                    + para_count
                    + (if msg_type == SRCPMessageType::SET {
                      1 //Bei SET braucht es noch den Value Wert zusätzlich
                    } else {
                      0
                    }))
                {
                  //Alles ausser Type müssen eine Zahl sein
                  result = true;
                  for i in 0..cmd_msg.parameter.len() {
                    if (i != 1) && cmd_msg.parameter[i].parse::<u32>().is_err() {
                      result = false;
                      self
                        .tx
                        .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
                        .unwrap();
                      break;
                    }
                  }
                  if result {
                    //OK an diese Session
                    self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
                  }
                } else {
                  self
                    .tx
                    .send(SRCPMessage::new_err(cmd_msg, "419", "list too short"))
                    .unwrap();
                }
              } else {
                self
                  .tx
                  .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
                  .unwrap();
              }
            } else {
              self
                .tx
                .send(SRCPMessage::new_err(cmd_msg, "419", "list too short"))
                .unwrap();
            }
          } else {
            self
              .tx
              .send(SRCPMessage::new_err(
                cmd_msg,
                "420",
                "unsupported device protocol",
              ))
              .unwrap();
          }
        }
      }
    }
    result
  }

  /// Empfangenes Kommando ausführen und versenden, ggf. interne Daten Updaten für späteren Refresh.
  /// Das Kommando muss gültig sein (validate_cmd), es wird hier nicht mehr überprüft.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn execute_cmd(&mut self, cmd_msg: &SRCPMessage) {
    let SRCPMessageID::Command { msg_type } = cmd_msg.message_id else {return};
    match msg_type {
      SRCPMessageType::INIT => {
        //Protokoll und Version für SM
        let (prot_name, prot_ver) = self
          .gl_ga_prot_names
          .get(cmd_msg.parameter[0].as_str())
          .unwrap();
        //Verlangtes Protokoll wird das aktive SM Protokoll
        self.sm_protokoll = Some((
          DdlProtokolle::from_str(prot_name).unwrap(),
          prot_ver.to_string(),
        ));
        //Und Protokoll SM INIT
        let (prot, prot_ver) = self.sm_protokoll.as_ref().unwrap();
        let protokoll = &self.all_protokolle[prot][prot_ver.as_str()];
        protokoll.borrow_mut().sm_init();
      }
      SRCPMessageType::TERM => {
        //Protokoll SM TERM
        let (prot, prot_ver) = self.sm_protokoll.as_ref().unwrap();
        let protokoll = &self.all_protokolle[prot][prot_ver.as_str()];
        protokoll.borrow_mut().sm_term();
        //Und kein aktives SM Protokoll mehr vorhanden
        self.sm_protokoll = None;
      }
      SRCPMessageType::GET => {
        //Alle (nach Type bis Schluss) notwendigen Parameter zu Vec<u32> konvertieren.
        let mut param: Vec<u32> = Vec::new();
        for p_str in &cmd_msg.parameter[2..] {
          param.push(p_str.parse::<u32>().unwrap());
        }
        //Protokoll für SM
        let (prot, prot_ver) = self.sm_protokoll.as_ref().unwrap();
        let protokoll = &self.all_protokolle[prot][prot_ver.as_str()];
        protokoll.borrow_mut().sm_read(
          cmd_msg.get_adr().unwrap(),
          &cmd_msg.parameter[1],
          &param,
          cmd_msg.session_id.unwrap(),
        );
      }
      SRCPMessageType::SET => {
        //Alle (nach Type bis Schluss - 1) notwendigen Parameter zu Vec<u32> konvertieren.
        let mut param: Vec<u32> = Vec::new();
        for p_str in &cmd_msg.parameter[2..cmd_msg.parameter.len() - 1] {
          param.push(p_str.parse::<u32>().unwrap());
        }
        //Der letzte Parameter ist der zu schreibende Wert
        let value = cmd_msg.parameter.last().unwrap().parse::<u32>().unwrap();
        //Protokoll für SM
        let (prot, prot_ver) = self.sm_protokoll.as_ref().unwrap();
        let protokoll = &self.all_protokolle[prot][prot_ver.as_str()];
        protokoll.borrow_mut().sm_write(
          cmd_msg.get_adr().unwrap(),
          &cmd_msg.parameter[1],
          &param,
          value,
          cmd_msg.session_id.unwrap(),
        );
      }
    }
  }

  /// Alle internen zustände als Info Message versenden
  /// # Arguments
  /// * session_id - SRCP Client Session ID an die die Zustände gesendet werden sollen.
  ///                None -> Info an alle SRCP Clients
  fn send_all_info(&self, _session_id: Option<u32>) {
    //SM hat keine internen Zustände die an alle SRCP Info Clients gensendet werden müssen
  }
}
