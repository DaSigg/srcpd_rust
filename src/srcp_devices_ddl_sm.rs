use std::{collections::HashMap, sync::mpsc::Sender};

use log::debug;

use crate::{
  srcp_devices_ddl::SRCPDeviceDDL,
  srcp_protocol_ddl::{DdlProtokolle, HashMapProtokollVersion, SmReadWrite, SmReadWriteType},
  srcp_server_types::{SRCPMessage, SRCPMessageDevice, SRCPMessageID, SRCPMessageType},
};

/// SM Device
pub struct DdlSM {
  //SRCP Bus auf dem gearbeitet wird
  bus: usize,
  //Sender für SRCP Antworten
  tx: Sender<SRCPMessage>,
  //Alle vorhandenen Protokollimplementierungen mit allen Versionen
  all_protokolle: HashMapProtokollVersion,
  //Konvertierung auf bei GL/GA verwendete (Protokollnamen, Version)
  gl_ga_prot_names: HashMap<String, (String, String)>,
  //Aktuell verwendetes SM Protokoll und Version, durch INIT gesetzt.
  sm_protokoll: Option<(DdlProtokolle, String)>,
  ///Für welche SM's soll ein Oszi Trigger ausgegeben werden?
  trigger: Vec<u32>,
}

impl DdlSM {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * bus - SRCP Bus auf dem dieses Device arbeitet
  /// * tx - Sender für Info Messages / Antworten an SRCP Clients
  /// * all_protokolle - Alle vorhandenen Protokollimplementierungen mit allen Versionen
  /// * trigger_adr - Oszi Trigger Adressen aus Konfigfile
  pub fn new(
    bus: usize, tx: Sender<SRCPMessage>, all_protokolle: HashMapProtokollVersion,
    trigger_adr: Option<String>,
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
    let mut result = DdlSM {
      bus,
      tx,
      all_protokolle,
      gl_ga_prot_names,
      sm_protokoll: None,
      trigger: vec![],
    };
    result.trigger = result.eval_trigger_config(trigger_adr);
    result
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
    //Für SM wird unterstützt: INIT, TERM, SET, GET
    if let SRCPMessageID::Command { msg_type } = cmd_msg.message_id {
      match msg_type {
        SRCPMessageType::INIT => {
          //Format ist INIT <bus> SM <protocol>
          //Für <protocol> wird aktuell "NMRA" und "MFX" unterstützt
          //Leider ist in der srcp Protkollspez. hier "NMRA" definiert, bei GL/GA ist "N" definiert :-(
          //Um alles gleich machen zu können übersetzen wir hier zurück auf die Definitionen bei GL/GA
          //Wenn NMRA verwendet wird:
          // - ohne weiteren Parameter SM für GL, Standardverhalten
          // - mit NMRA 1 GA wird SM für GA gestartet (Einfache Zubehördecoder).
          // - mit NMRA 2 GA wird SM für GA gestartet (Erweiterte Zubehördecoder).
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
        SRCPMessageType::SET | SRCPMessageType::GET | SRCPMessageType::VERIFY => {
          //Format ist SET <bus> SM <decoderaddress> <type> <values ...> <set value>
          //<type> ist Protokollabhängig (z.B. bei NMRA CV, CVBIT, bei MFX CAMFX)
          //Anzahl weitere Parameter ist auch Protokollabhängig (z.B. NMRA CV: CV, Value, bei MFX CAMFX Block, CA, CAIndex, Index, Value)
          //<set value> nur bei SET und VERIFY, nicht bei GET
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
                    + (if msg_type == SRCPMessageType::GET {
                      0
                    } else {
                      1 //Bei SET und VERIFY braucht es noch den Value Wert zusätzlich
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
  /// * power - true wenn Power eingeschaltet, Booster On sind
  fn execute_cmd(&mut self, cmd_msg: &SRCPMessage, power: bool) {
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
          //Wenn Protokollversion über Init Befehl definiert wurde, dann diese verwenden
          if cmd_msg.parameter.len() > 1 {
            cmd_msg.parameter[1].clone()
          }
          else {
            prot_ver.to_string() //Default
          }
          ,
        ));
        //Und Protokoll SM INIT. Wenn neben Protokoll noch ein Parameter vorhanden ist, dann wird er sm_init übergeben.
        //zB
        // - mit NMRA 1 GA wird SM für GA gestartet (Einfache Zubehördecoder).
        // - mit NMRA 2 GA wird SM für GA gestartet (Erweiterte Zubehördecoder).
        let (prot, prot_ver) = self.sm_protokoll.as_ref().unwrap();
        let protokoll = &self.all_protokolle[prot][prot_ver.as_str()];
        protokoll.borrow_mut().sm_init(if cmd_msg.parameter.len() > 2 {Some(cmd_msg.parameter[2].as_str())} else {None});
        //SRCP Antwort OK zurücksenden
        let ok_msg = SRCPMessage::new_ok(cmd_msg, "200");
        self.tx.send(ok_msg).unwrap();
      }
      SRCPMessageType::TERM => {
        //Protokoll SM TERM
        let (prot, prot_ver) = self.sm_protokoll.as_ref().unwrap();
        let protokoll = &self.all_protokolle[prot][prot_ver.as_str()];
        protokoll.borrow_mut().sm_term();
        //Und kein aktives SM Protokoll mehr vorhanden
        self.sm_protokoll = None;
        //SRCP Antwort OK zurücksenden
        let ok_msg = SRCPMessage::new_ok(cmd_msg, "200");
        self.tx.send(ok_msg).unwrap();
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
        protokoll.borrow_mut().sm_read_write(&SmReadWrite {
          adr: cmd_msg.get_adr().unwrap(),
          prog_gleis: !power, //Prog.Gleismodus wenn Power aus
          sm_type: cmd_msg.parameter[1].clone(),
          para: param,
          val: SmReadWriteType::Read,
          session_id: cmd_msg.session_id.unwrap(),
          trigger: self.trigger.contains(cmd_msg.get_adr().as_ref().unwrap()),
        });
      }
      SRCPMessageType::SET | SRCPMessageType::VERIFY => {
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
        protokoll.borrow_mut().sm_read_write(&SmReadWrite {
          adr: cmd_msg.get_adr().unwrap(),
          prog_gleis: !power, //Prog.Gleismodus wenn Power aus
          sm_type: cmd_msg.parameter[1].clone(),
          para: param,
          val: if msg_type == SRCPMessageType::SET {
            SmReadWriteType::Write(value)
          } else {
            SmReadWriteType::Verify(value)
          },
          session_id: cmd_msg.session_id.unwrap(),
          trigger: self.trigger.contains(cmd_msg.get_adr().as_ref().unwrap()),
        });
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
  /// Muss zyklisch aufgerufen werden. Erlaubt dem Device die Ausführung von
  /// von neuen Kommando oder refresh unabhängigen Aufgaben.
  /// Liefert immer false, es wurde hier nie ein DDL Tel. versendet.
  /// Hier: Wenn vorhanden SM Info-Antwort Messages als srcp Message zurück senden
  /// # Arguments
  /// * power - true: Power / Booster ist ein, Strom auf den Schienen
  ///           false: Power / Booster ist aus
  fn execute(&mut self, _power: bool) -> bool {
    for (_, prot_familie) in &self.all_protokolle {
      for (_, prot) in prot_familie {
        if let Some(ans) = prot.borrow_mut().sm_get_answer() {
          debug!("SM RX Antwort: {:?}", ans);
          let mut srcp_para: Vec<String> = Vec::new();
          //Paramater zu SM sind: adr sm_type <alle paramater> value
          srcp_para.push(ans.adr.to_string());
          srcp_para.push(ans.sm_type);
          for p in ans.para {
            srcp_para.push(p.to_string());
          }
          let srcp_message = if let SmReadWriteType::ResultOk(val) = ans.val {
            //OK Message
            srcp_para.push(val.to_string());
            SRCPMessage {
              session_id: Some(ans.session_id),
              bus: self.bus,
              message_id: SRCPMessageID::Ok {
                ok_code: "200".to_string(),
              },
              device: SRCPMessageDevice::SM,
              parameter: srcp_para,
            }
          } else {
            //Error
            SRCPMessage {
              session_id: Some(ans.session_id),
              bus: self.bus,
              message_id: SRCPMessageID::Err {
                err_code: "412".to_string(),
                err_text: "wrong value".to_string(),
              },
              device: SRCPMessageDevice::SM,
              parameter: srcp_para,
            }
          };
          debug!("SM Antwort: {}", srcp_message.to_string());
          self.tx.send(srcp_message).unwrap();
        }
      }
    }
    false
  }
}
