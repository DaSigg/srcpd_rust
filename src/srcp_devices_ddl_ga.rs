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
  value: [usize; 2],
  //Gewähltes Protokoll
  protokoll: DdlProtokolle,
  //Optional: Protokoll Version
  protokoll_version: Option<String>,
  //Oszi Trigger?
  trigger: bool,
}
impl GAInit {
  fn new(protokoll: DdlProtokolle, protokoll_version: Option<String>, trigger: bool) -> GAInit {
    GAInit {
      value: [0, 0],
      protokoll,
      protokoll_version,
      trigger,
    }
  }
}

///Grund für GA in "GADelay"
enum GADelayGrund {
  ///Einschaltung war noch nicht möglich weil auf dem gleichen Dekoder noch eine andere Ausgabe aktiv war
  ///value: wie lange muss der Ausgang aktiv bleiben
  Einschalten(Duration),
  ///Verzögertes Ausschalten
  ///value: wann soll der Ausgang ausgeschaltet werden
  Ausschalten(Instant),
}
///Verwaltung verzögerte Ausgabe und automatisches Ausschalten nach Delay
struct GADelay {
  adr: u32,
  port: usize,
  ga_delay_grund: GADelayGrund,
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
  all_ga: HashMap<u32, GAInit>,
  //Verwaltung aller GA die verzögert ausgegeben oder nach Delayzeit noch automatisch ausgeschaltet werden müssen
  all_ga_delay: Vec<GADelay>,
  ///Für welche GL's soll ein Oszi Trigger ausgegeben werden?
  trigger: Vec<u32>,
  ///Und Port für Oszi trigger
  trigger_port: Option<u32>,
}

impl DdlGA<'_> {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * bus - SRCP Bus auf dem dieses Device arbeitet
  /// * tx - Sender für Info Messages / Antworten an SRCP Clients
  /// * spidev - geöffnetes Spidev zur Ausgabe an Booster
  /// * all_protokolle - Alle vorhandenen Protokollimplementierungen mit allen Versionen
  /// * trigger_port - Oszi Triggerport aus Konfigfile
  /// * trigger_adr - Oszi Trigger Adressen aus Konfigfile
  pub fn new(
    bus: usize, tx: Sender<SRCPMessage>, spidev: &Option<Spidev>,
    all_protokolle: HashMapProtokollVersion, trigger_port: Option<String>,
    trigger_adr: Option<String>,
  ) -> DdlGA<'_> {
    let mut result = DdlGA {
      bus,
      tx,
      spidev,
      all_protokolle,
      all_ga: HashMap::new(),
      all_ga_delay: Vec::new(),
      trigger: vec![],
      trigger_port: None,
    };
    result.trigger_port = result.eval_trigger_port_config(trigger_port);
    result.trigger = result.eval_trigger_config(trigger_adr);
    result
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
      if let Ok(adr) = cmd_msg.parameter[0].parse::<u32>() {
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
  fn send_info_msg(&self, session_id: Option<u32>, adr: u32, port: usize, value: usize) {
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
          value.to_string(),
        ],
      ))
      .unwrap();
  }

  /// GA Port Ausgänge senden und Zustand speichern
  /// Liefert true zurück, wenn Timeout zur automatischen Abschaltung durch Protokoll / Dekoder übernommen wird.
  /// # Arguments
  /// * adr - GA Adresse
  /// * port - GA Port
  /// * value - Gewünschter Output Zustand
  /// * timeout - Wenn das Protokoll eine automatische Ausschaltung des Ausgangs durch den Dekoder unterstützt kann hier die Zeit in ms angegeben werden.
  ///             None = kein Timeout, dauerhaft schalten. 
  ///             Duration::ZERO = Port ignorieren, Value ist der zu sendende Begriff (z.B. Erweiterte Funktionsdekoder NMRA/DCC Signalbegriff)
  fn send_ga(&mut self, adr: u32, port: usize, value: usize, timeout: Option<Duration>) -> bool {
    let ga = self.all_ga.get_mut(&adr).unwrap();
    //Neuen Zustand speichern
    ga.value[port] = value;
    //Zum Booster Versenden, wenn keine Version angegeben ist das Default-Protokoll verwenden, ansonsten verlangte Version
    let prot_ver = if let Some(protokoll_version) = &ga.protokoll_version {
      protokoll_version
    }
    else {
      let mut def_ver_gefunden: Option<&str> = None;
      for (ver, prot) in &self.all_protokolle[&ga.protokoll] {
        if prot.borrow().is_default() {
          def_ver_gefunden = Some(ver);
        }
      }
      if let Some(def_ver) = def_ver_gefunden {
        def_ver
      }
      else {
        panic!("Für Protokoll {:?} ist keine Default Version vorhanden.", ga.protokoll);
      }
    };
    let protokoll = self.all_protokolle[&ga.protokoll].get(prot_ver).unwrap();
    let mut ddl_tel = protokoll.borrow().get_ga_new_tel(adr, ga.trigger);
    let result = protokoll
      .borrow_mut()
      .get_ga_tel(adr, port, value, timeout, &mut ddl_tel);
    //Es ist nur ein Telegramm, keine Behandlung verzögertes Senden notwendig
    <DdlGA<'_> as SRCPDeviceDDL>::send(self.spidev, &mut ddl_tel, self.trigger_port);
    //Alle Info Clients über neuen Zustand Informieren
    self.send_info_msg(None, adr, port, value);
    return result;
  }

  /// Stellt fest ob ein Dekoder bereits eine aktive Ausgabe hat für alle GA's, die über SRCP automatisch nach
  /// Timout in SET GA Kommando ausgeschaltet werden.
  /// # Arguments
  /// * adr - GA Adresse
  fn is_dekoder_aktiv(&self, ga_adr: u32) -> bool {
    let dek_adr = (ga_adr - 1) / 4;
    //Durchsuchen ob für diesen Dekoder eine Ausschaltung hängig ist
    for ga_delay in &self.all_ga_delay {
      match ga_delay.ga_delay_grund {
        GADelayGrund::Einschalten(_) => (),
        GADelayGrund::Ausschalten(_) => {
          if dek_adr == ((ga_delay.adr - 1) / 4) {
            //Dekoder bereits aktiv
            return true;
          }
        }
      }
    }
    return false;
  }

  /// GA einschalten mit Timeout für automatische Ausschaltung
  /// # Arguments
  /// * adr - GA Adresse
  /// * port - Port zu Adresse
  /// * timeout - Nach welcher Zeit soll die automatische Ausschaltung erfolgen
  fn set_ga_on_timeout(&mut self, adr: u32, port: usize, timeout: Duration) {
    //Einschalten ausführen
    if ! self.send_ga(adr, port, 1, Some(timeout)) {
      //In Verwaltung zur automatischen Ausschaltung übernehmen
      self.all_ga_delay.push(GADelay {
        adr,
        port,
        ga_delay_grund: GADelayGrund::Ausschalten(Instant::now() + timeout),
      });
    }
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
    //Für GA wird unterstützt: INIT, TERM, SET, GET
    if let SRCPMessageID::Command { msg_type } = cmd_msg.message_id {
      match msg_type {
        SRCPMessageType::INIT => {
          //Format ist INIT <bus> GA <addr> <protocol> <optional further parameters>
          //Zwei Parameter müssen vorhanden sein: <addr> <protocol>
          //<optional further parameters> für Protokoll "N" ist "protocolversion".
          // "1" = GA "Einfache Zubehördecoder", Default wenn keine Angabe
          // "2" = GA "Erweiterte Zubehördecoder"
          if cmd_msg.parameter.len() >= 2 {
            //Zuerst das Protokoll
            if let Some(protokoll) = DdlProtokolle::from_str(cmd_msg.parameter[1].as_str()) {
              if let Some(protokolle_impl) = self.all_protokolle.get(&protokoll) {
                //Keine Protokollversionsangabe bei INIT GA -> immer die erste vorhandene Version verwenden
                //Wenn <optional further parameters> für Protokoll "N" angegeben ist"X" für "Erweiterte Zubehördecoder"
                let prot_version = if cmd_msg.parameter.len() >= 3 {
                  cmd_msg.parameter[2].as_str()
                }
                else {
                  "1"
                };
                //Protokollversion ist angebeben, muss "1" oder "2" sein
                if prot_version != "1" && prot_version != "2" {
                  self
                    .tx
                    .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
                    .unwrap();
                }
                else {
                  let prot_impl = protokolle_impl.get(prot_version).unwrap();
                  //Adressprüfung
                  if let Ok(adr) = cmd_msg.parameter[0].parse::<u32>() {
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
          if let Ok(adr) = cmd_msg.parameter[0].parse::<u32>() {
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
            if cmd_msg.parameter[2].parse::<u8>().is_ok()
              && cmd_msg.parameter[3].parse::<i32>().is_ok()
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
        SRCPMessageType::VERIFY => {
          //Verify wird für GA's nicht unterstützt
          self
            .tx
            .send(SRCPMessage::new_err(
              cmd_msg,
              "423",
              "unsupported operation",
            ))
            .unwrap();
        }
      };
    }
    result
  }

  /// Empfangenes Kommando ausführen und versenden, ggf. interne Daten Updaten für späteren Refresh.
  /// Das Kommando muss gültig sein (validate_cmd), es wird hier nicht mehr überprüft.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  /// * power - true wenn Power eingeschaltet, Booster On sind
  fn execute_cmd(&mut self, cmd_msg: &SRCPMessage, _power: bool) {
    let SRCPMessageID::Command { msg_type } = cmd_msg.message_id else {
      return;
    };
    match msg_type {
      SRCPMessageType::INIT => {
        //Format ist INIT <bus> GA <addr> <protocol> <optional further parameters>
        //Zwei Parameter müssen vorhanden sein: <addr> <protocol>
        //Zuerst das Protokoll
        let Some(protokoll) = DdlProtokolle::from_str(cmd_msg.parameter[1].as_str()) else {
          return;
        };
        //Adresse
        let adr = cmd_msg.parameter[0].parse::<u32>().unwrap();
        self
          .all_ga
          .insert(adr, GAInit::new(protokoll, if cmd_msg.parameter.len() >= 3 {Some(cmd_msg.parameter[2].clone())} else {None}, self.trigger.contains(&adr)));
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
        let adr = cmd_msg.parameter[0].parse::<u32>().unwrap();
        self.all_ga.remove(&adr);
      }
      SRCPMessageType::GET => {
        //Format ist GET <bus> GA <addr> <port>
        let adr = cmd_msg.parameter[0].parse::<u32>().unwrap();
        let port = cmd_msg.parameter[1].parse::<usize>().unwrap();
        let ga = &self.all_ga[&adr];
        //INFO <bus> GA <adr> <port> <value>
        self.send_info_msg(cmd_msg.session_id, adr, port, ga.value[port]);
      }
      SRCPMessageType::SET => {
        let adr = cmd_msg.parameter[0].parse::<u32>().unwrap();
        //Da SET verzögert über Queue ausgeführt wird könnte ein TERM dazwischen gekommen sein, Adresse nochmals prüfen
        if self.all_ga.contains_key(&adr) {
          let port = cmd_msg.parameter[1].parse::<usize>().unwrap();
          let value = cmd_msg.parameter[2].parse::<usize>().unwrap();
          let switch_off_timeout = cmd_msg.parameter[3].parse::<i32>().unwrap();
          if (value != 0) && (switch_off_timeout > 0) {
            //Zumindest die alten Märklin k83 Dekoder könne nicht mehrere Ausgänge gleichzeitig aktiviert haben.
            //Wenn Ausschalten hier gemacht wird, dann stellen wir hier auch sicher, dass nicht mehr als ein
            //Ausgang auf einem Dekoder gleichzeitg aktiv ist.
            //Wenn der Anwender das übernimmt (Zeit <=0), dann muss er das elbst im Griff haben
            if self.is_dekoder_aktiv(adr) {
              //In Verwaltung für verzögertes Einschalten übernehmen
              self.all_ga_delay.push(GADelay {
                adr,
                port,
                ga_delay_grund: GADelayGrund::Einschalten(Duration::from_millis(
                  switch_off_timeout.try_into().unwrap(),
                )),
              });
            } else {
              self.set_ga_on_timeout(
                adr,
                port,
                Duration::from_millis(switch_off_timeout.try_into().unwrap()),
              );
            }
          } else {
            //Keine Zeitangabe für Ausschalten vom Anwender oder explizites Ausschalten, immer sofort ausführen
            self.send_ga(adr, port, value, None);
          }
        }
      }
      SRCPMessageType::VERIFY => {
        //Verify wird für GA's nicht unterstützt, wurde bei Validate bereits abgelehnt
      }
    }
  }

  /// Alle internen Zustände als Info Message versenden
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
  /// von neuen Kommandos oder refresh unabhängigen Aufgaben.
  /// Liefert true zurück, wenn durch den Aufruf min. ein DDL Telegramm gesendet wurde, sonst false.
  /// Hier wird das verzögerte Einschaloten und automatische Ausschalten von GA Outputs nach Delay Zeit ausgeführt
  /// # Arguments
  /// * power - true: Power / Booster ist ein, Strom auf den Schienen
  ///           false: Power / Booster ist aus
  fn execute(&mut self, power: bool) -> bool {
    let mut tel_gesendet = false;
    //Ein- Ausschaltkommando senden macht nur Sinn, wenn Power vorhanden ist
    if power {
      let mut i = 0;
      while i < self.all_ga_delay.len() {
        let ga_delay = &self.all_ga_delay[i];
        match ga_delay.ga_delay_grund {
          GADelayGrund::Einschalten(einschaltzeit) => {
            //Falls der Dekoder nicht mehr verwendet wird kann nun die Ausgabe dieses GA Kommandos erfolgen
            if !self.is_dekoder_aktiv(ga_delay.adr) {
              //Ausführen
              self.set_ga_on_timeout(ga_delay.adr, ga_delay.port, einschaltzeit);
              //Eintrag löschen
              self.all_ga_delay.remove(i);
            } else {
              i += 1;
            }
          }
          GADelayGrund::Ausschalten(off_zeit) => {
            if Instant::now() > off_zeit {
              //Auto off
              tel_gesendet = true;
              self.send_ga(ga_delay.adr, ga_delay.port, 0, None);
              self.all_ga_delay.remove(i);
            } else {
              i += 1;
            }
          }
        }
      }
    }
    tel_gesendet
  }
}
