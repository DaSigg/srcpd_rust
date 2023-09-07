use std::{
  collections::HashMap,
  sync::mpsc::Sender,
  time::{Duration, Instant},
};

use spidev::Spidev;

use crate::{
  srcp_devices_ddl::SRCPDeviceDDL,
  srcp_protocol_ddl::{DdlProtokolle, HashMapProtokollVersion},
  srcp_server_types::{SRCPMessage, SRCPMessageDevice, SRCPMessageID, SRCPMessageType},
};

///Verwaltung eines initialisierten GA's
struct GAInit {
  //Aktuelles Value Pro Port.
  //Aktuell mit DLL unterstützte Protokolle DCC und MM haben nur immer 2 Ports auf einer Adresse
  value: [bool; 2],
  //Gewähltes Protokoll
  protokoll: DdlProtokolle,
}
impl GAInit {
  fn new(protokoll: DdlProtokolle) -> GAInit {
    GAInit {
      value: [false, false],
      protokoll,
    }
  }
}

///Verwaltung automatisches Ausschalten nach Delay
struct GAAutoOff {
  adr: usize,
  port: usize,
  off_zeit: Instant,
}

pub struct DdlGA<'a> {
  //SRCP Bus auf dem gearbeitet wird
  bus: usize,
  //Sender für SRCP Antworten
  tx: Sender<SRCPMessage>,
  //SPI Bus für Ausgabe
  spidev: &'a Option<Spidev>,
  //Alle vorhandenen Protokollimplementierungen mit allen Versionen
  all_protokolle: HashMapProtokollVersion,
  //Alle initialisierten GA, Key Adresse
  all_ga: HashMap<usize, GAInit>,
  //Verwaltung aller GA die nach Delayzeit noch automatisch ausgeschaltet werden müssen
  all_ga_auto_off: Vec<GAAutoOff>,
}

impl DdlGA<'_> {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * bus - SRCP Bus auf dem dieses Device arbeitet
  /// * tx - Sender für Info Messages / Antworten an SRCP Clients
  /// * spidev - geöffnetes Spidev zur Ausgabe an Booster
  /// * all_protokolle - Alle vorhandenen Protokollimplementierungen mit allen Versionen
  pub fn new(
    bus: usize, tx: Sender<SRCPMessage>, spidev: &Option<Spidev>,
    all_protokolle: HashMapProtokollVersion,
  ) -> DdlGA {
    DdlGA {
      bus,
      tx,
      spidev,
      all_protokolle,
      all_ga: HashMap::new(),
      all_ga_auto_off: Vec::new(),
    }
  }

  /// GET und SET (ohne Values für SET) validieren
  /// return true wenn OK.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  /// * anz_parameter - Min. Anzahl notwendige Parameter (2 für GET, 4 für SET)
  fn validate_get_set(&self, cmd_msg: &SRCPMessage, anz_parameter: usize) -> bool {
    let mut result = false;
    //Format ist GET <bus> GA <addr> <port>
    //Format ist SET <bus> GA <addr> <port> <value> <time>
    if cmd_msg.parameter.len() >= anz_parameter {
      if let Ok(adr) = cmd_msg.parameter[0].parse::<usize>() {
        if let Some(ga) = self.all_ga.get(&adr) {
          //Wenn Adr initialisiert ist muss noch Port gültig sein
          if let Ok(port) = cmd_msg.parameter[1].parse::<usize>() {
            if port < ga.value.len() {
              result = true;
            } else {
              self
                .tx
                .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
                .unwrap();
            }
          } else {
            self
              .tx
              .send(SRCPMessage::new_err(cmd_msg, "416", "no data"))
              .unwrap();
          }
        } else {
          self
            .tx
            .send(SRCPMessage::new_err(cmd_msg, "416", "no data"))
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
    result
  }

  /// INFO Message versenden
  /// # Arguments
  /// * session_id - None: an alle SRCP Info Clients, sonst nur an den mit SessionID
  /// * adr - GA Adresse
  /// * port - GA Port
  /// * value - GA Port Zustand
  fn send_info_msg(&self, session_id: Option<u32>, adr: usize, port: usize, value: bool) {
    //INFO <bus> GA <adr> <port> <value>
    self
      .tx
      .send(SRCPMessage::new(
        session_id,
        self.bus,
        SRCPMessageID::Info {
          info_code: "100".to_string(),
        },
        SRCPMessageDevice::GA,
        vec![
          adr.to_string(),
          port.to_string(),
          if value {
            "1".to_string()
          } else {
            "0".to_string()
          },
        ],
      ))
      .unwrap();
  }

  /// GA Port Ausgänge senden und Zustand speichern
  /// # Arguments
  /// * adr - GA Adresse
  /// * port - GA Port
  /// * value - Gewünschter Output Zustand
  fn send_ga(&mut self, adr: usize, port: usize, value: bool) {
    //Neuen Zustand speichern
    self.all_ga.get_mut(&adr).unwrap().value[port] = value;
    let protokoll = self.all_ga[&adr].protokoll;
    //Zum Booster Versenden, erstes passendes Protokoll verwenden, keine Versionsangabe für GA
    let protokoll = self.all_protokolle[&protokoll].values().next().unwrap();
    let mut ddl_tel = protokoll.borrow().get_ga_new_tel(adr);
    protokoll
      .borrow_mut()
      .get_ga_tel(adr, port, value, &mut ddl_tel);
    //Es ist nur ein Telegramm, keine Behandlung verzögertes senden notwendig
    <DdlGA<'_> as SRCPDeviceDDL>::send(self.spidev, &mut ddl_tel);
    //Alle Info Clients über neuen Zustand Informieren
    self.send_info_msg(None, adr, port, value);
  }
}

impl SRCPDeviceDDL for DdlGA<'_> {
  /// Empfangenes Kommando validieren
  /// Return true wenn Ok.
  /// Sendet die Antwort Message (Ok / Err) an Sender zurück.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn validate_cmd(&self, cmd_msg: &SRCPMessage) -> bool {
    let mut result = false;
    //Für GA wird unterstützt: INIT, SET, GET
    if let SRCPMessageID::Command { msg_type } = cmd_msg.message_id {
      match msg_type {
        SRCPMessageType::INIT => {
          //Format ist INIT <bus> GA <addr> <protocol> <optional further parameters>
          //Zwei Parameter müssen vorhanden sein: <addr> <protocol>
          if cmd_msg.parameter.len() >= 2 {
            //Zuerst das Protokoll
            if let Some(protokoll) = DdlProtokolle::from_str(cmd_msg.parameter[1].as_str()) {
              if let Some(protokolle_impl) = self.all_protokolle.get(&protokoll) {
                //Keine Protokollversionsangabe bei INIT GA -> immer die erste vorhandene Version verwenden
                let prot_impl = protokolle_impl.values().next().unwrap();
                //Adressprüfung
                if let Ok(adr) = cmd_msg.parameter[0].parse::<usize>() {
                  if (adr > 0) && (adr <= prot_impl.borrow_mut().get_ga_max_adr()) {
                    //OK an diese Session
                    self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
                    result = true;
                  } else {
                    self
                      .tx
                      .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
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
          } else {
            self
              .tx
              .send(SRCPMessage::new_err(cmd_msg, "419", "list too short"))
              .unwrap();
          }
        }
        SRCPMessageType::TERM => {
          //Format ist TERM <bus> GA <addr>
          //Adressprüfung
          if let Ok(adr) = cmd_msg.parameter[0].parse::<usize>() {
            if self.all_ga.contains_key(&adr) {
              //OK an diese Session
              self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
              result = true;
            } else {
              self
                .tx
                .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
                .unwrap();
            }
          } else {
            self
              .tx
              .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
              .unwrap();
          }
        }
        SRCPMessageType::GET => {
          //Format ist GET <bus> GA <addr> <port>
          if self.validate_get_set(cmd_msg, 2) {
            result = true;
          }
        }
        SRCPMessageType::SET => {
          //Format ist SET <bus> GA <addr> <port> <value> <time>
          if self.validate_get_set(cmd_msg, 4) {
            //Jetzt noch <value> und <time> prüfen
            if (cmd_msg.parameter[2] == "0" || cmd_msg.parameter[2] == "1")
              && cmd_msg.parameter[3].parse::<i16>().is_ok()
            {
              //OK an diese Session
              self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
              result = true;
            } else {
              self
                .tx
                .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
                .unwrap();
            }
          }
        }
      };
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
        //Format ist INIT <bus> GA <addr> <protocol> <optional further parameters>
        //Zwei Parameter müssen vorhanden sein: <addr> <protocol>
        //Zuerst das Protokoll
        let Some(protokoll) = DdlProtokolle::from_str(cmd_msg.parameter[1].as_str()) else {return};
        //Adresse
        let adr = cmd_msg.parameter[0].parse::<usize>().unwrap();
        self.all_ga.insert(adr, GAInit::new(protokoll));
        //INFO <bus> GA <adr> <protokoll>
        self
          .tx
          .send(SRCPMessage::new(
            None,
            cmd_msg.bus,
            SRCPMessageID::Info {
              info_code: "100".to_string(),
            },
            cmd_msg.device.clone(),
            cmd_msg.parameter.clone(),
          ))
          .unwrap();
      }
      SRCPMessageType::TERM => {
        //Format ist TERM <bus> GA <addr>
        //Adresse
        let adr = cmd_msg.parameter[0].parse::<usize>().unwrap();
        self.all_ga.remove(&adr);
      }
      SRCPMessageType::GET => {
        //Format ist GET <bus> GA <addr> <port>
        let adr = cmd_msg.parameter[0].parse::<usize>().unwrap();
        let port = cmd_msg.parameter[1].parse::<usize>().unwrap();
        let ga = &self.all_ga[&adr];
        //INFO <bus> GA <adr> <port> <value>
        self.send_info_msg(cmd_msg.session_id, adr, port, ga.value[port]);
      }
      SRCPMessageType::SET => {
        let adr = cmd_msg.parameter[0].parse::<usize>().unwrap();
        //Da SET verzögert über Queue ausgeführt wird könnte ein TERM dazwischen gekommen sein, Adresse nochmals prüfen
        if self.all_ga.contains_key(&adr) {
          let port = cmd_msg.parameter[1].parse::<usize>().unwrap();
          let value = cmd_msg.parameter[2] == "1";
          self.send_ga(adr, port, value);
          let switch_off_timeout = cmd_msg.parameter[3].parse::<i16>().unwrap();
          if switch_off_timeout > 0 {
            //In Verwaltung zur automatischen Ausschaltung übernehmen
            self.all_ga_auto_off.push(GAAutoOff {
              adr,
              port,
              off_zeit: Instant::now()
                + Duration::from_millis(switch_off_timeout.try_into().unwrap()),
            });
          }
        }
      }
    };
  }

  /// Alle internen zustände als Info Message versenden
  /// # Arguments
  /// * session_id - SRCP Client Session ID an die die Zustände gesendet werden sollen.
  ///                None -> Info an alle SRCP Clients
  fn send_all_info(&self, session_id: Option<u32>) {
    //Über alle initialisierten GA's
    for (adr, ga) in &self.all_ga {
      //Über alle Ports dieses GA's
      for port in 0..ga.value.len() {
        self.send_info_msg(session_id, *adr, port, ga.value[port]);
      }
    }
  }

  /// Muss zyklisch aufgerufen werden. Erlaubt dem Device die Ausführung von
  /// von neuen Kommando oder refresh unabhängigen Aufgaben.
  /// Hier wird das automatische Ausschalten von GA Outputs nach Delay Zeit ausgeführt
  /// # Arguments
  /// * power - true: Power / Booster ist ein, Strom auf den Schienen
  ///           false: Power / Booster ist aus
  fn execute(&mut self, power: bool) {
    //Ausschaltkommando senden macht nur Sinn, wenn Power vorhanden ist
    if power {
      let mut i = 0;
      while i < self.all_ga_auto_off.len() {
        let ga_auto_off = &self.all_ga_auto_off[i];
        if Instant::now() > ga_auto_off.off_zeit {
          //Auto off
          self.send_ga(ga_auto_off.adr, ga_auto_off.port, false);
          self.all_ga_auto_off.remove(i);
        } else {
          i += 1;
        }
      }
    }
  }
}
