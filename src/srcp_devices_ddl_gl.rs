use std::{collections::HashMap, sync::mpsc::Sender, thread, time::Instant};

use spidev::Spidev;

use crate::{
  srcp_devices_ddl::SRCPDeviceDDL,
  srcp_protocol_ddl::{DdlProtokolle, DdlTel, GLDriveMode, HashMapProtokollVersion},
  srcp_server_types::{SRCPMessage, SRCPMessageDevice, SRCPMessageID, SRCPMessageType},
};

/// Anzahl initialisierter GL damit in Modus ohne extra Delays ziwschen Telegrammen
/// an die selbe GL  gewechselt wird.
/// Dies muss genügend gross sein, um einen verzögert gesendeten Buffer einer GL innerhalb eines
/// Refreshzyklus abzubauen.
/// Theoretischer Worst Case: DCC Lok mit 64 Funktionen könnte zu 11 Telegrammen führen
const MIN_ANZ_GL_NO_DELAY: usize = 15;

///Verwaltung einer initialisierten GL's
struct GLInit {
  //Aktuelles Fahrtrichtung
  direction: GLDriveMode,
  //Aktuelle Geschwindigkeit
  speed: usize,
  //Zusatzfunktionen
  fnkt: u64,
  //Gewähltes Protokoll
  protokoll: DdlProtokolle,
  //Gewählte Protokollversion
  protokoll_version: String,
  //Gewählte Anzahl v-Stufen
  protokoll_speedsteps: usize,
  //Anzahl Funktionen
  protokoll_number_functions: usize,
}
impl GLInit {
  fn new(
    protokoll: DdlProtokolle, protokoll_version: String, protokoll_speedsteps: usize,
    protokoll_number_functions: usize,
  ) -> GLInit {
    GLInit {
      protokoll,
      protokoll_version,
      protokoll_speedsteps,
      protokoll_number_functions,
      direction: GLDriveMode::Vorwaerts,
      speed: 0,
      fnkt: 0,
    }
  }
}

pub struct DdlGL<'a> {
  ///SRCP Bus auf dem gearbeitet wird
  bus: usize,
  ///Sender für SRCP Antworten
  tx: Sender<SRCPMessage>,
  ///SPI Bus für Ausgabe
  spidev: &'a Option<Spidev>,
  ///Alle vorhandenen Protokollimplementierungen mit allen Versionen
  all_protokolle: HashMapProtokollVersion,
  ///Alle initialisierten GL, Key Adresse
  all_gl: HashMap<usize, GLInit>,
  ///Letzte GL Adr. die im Refreshzyklus war. 0 solange keine GL vorhanden ist.
  adr_refresh: usize,
  ///Alle noch nicht durch GL verwendeten aber vorhandenen Protokolle für Idle Telegramme
  all_idle_protokolle: Vec<DdlProtokolle>,
  ///Buffer für verzögertes senden
  tel_buffer: Option<DdlTel>,
}

impl DdlGL<'_> {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * bus - SRCP Bus auf dem dieses Device arbeitet
  /// * tx - Sender für Info Messages / Antworten an SRCP Clients
  /// * spidev - geöffnetes Spidev zur Ausgabe an Booster
  /// * all_protokolle - Alle vorhandenen Protokollimplementierungen mit allen Versionen
  pub fn new(
    bus: usize, tx: Sender<SRCPMessage>, spidev: &Option<Spidev>,
    all_protokolle: HashMapProtokollVersion,
  ) -> DdlGL {
    let mut all_idle_protokolle: Vec<DdlProtokolle> = Vec::new();
    //Zuerst sind mal alle Protokolle nicht verwendet
    for (protokoll, _) in &all_protokolle {
      all_idle_protokolle.push(*protokoll);
    }
    DdlGL {
      bus,
      tx,
      spidev,
      all_protokolle,
      all_gl: HashMap::new(),
      adr_refresh: 0,
      all_idle_protokolle,
      tel_buffer: None,
    }
  }

  /// GET und SET (ohne Values für SET) validieren
  /// return true wenn OK.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  /// * anz_parameter - Min. Anzahl notwendige Parameter (1 für GET, 4 für SET)
  fn validate_get_set(&self, cmd_msg: &SRCPMessage, anz_parameter: usize) -> bool {
    let mut result = false;
    //Format ist GET <bus> GL <addr>
    //Format ist SET <bus> GL <addr> <drivemode> <V> <V_max> <f0> . . <fn>
    if cmd_msg.parameter.len() >= anz_parameter {
      if let Ok(adr) = cmd_msg.parameter[0].parse::<usize>() {
        if let Some(_) = self.all_gl.get(&adr) {
          result = true;
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
  /// * adr - GA Adresse -> Muss gültig, d.h. initialisiert sein
  fn send_info_msg(&self, session_id: Option<u32>, adr: usize) {
    //INFO <bus> GL <addr> <drivemode> <V> <V_max> <f0> . . <fn>
    let Some(gl) = self.all_gl.get(&adr) else {return;};
    let mut param: Vec<String> = vec![
      adr.to_string(),
      gl.direction.to_string(),
      gl.speed.to_string(),
      gl.protokoll_speedsteps.to_string(),
    ];
    for i in 0..gl.protokoll_number_functions {
      param.push((if (gl.fnkt & (1 << i)) == 0 { "0" } else { "1" }).to_string());
    }
    self
      .tx
      .send(SRCPMessage::new(
        session_id,
        self.bus,
        SRCPMessageID::Info {
          info_code: "100".to_string(),
        },
        SRCPMessageDevice::GL,
        param,
      ))
      .unwrap();
  }

  /// GL senden und Zustand speichern
  /// # Arguments
  /// * adr - GA Adresse
  /// * drivemode - Richtung / Nothalt
  /// * v - Gewünschte Geschwindigkeit von 0 bis v_max. v_max -> max. Speed gemäss init
  /// * v_max - Skalierung 100% v
  /// * funktionen - f0 bis fn als Bits
  /// * refresh - Aufruf wegen Senden aus Refreshzyklus -> immer alles senden, auch wenn keine Fx Veränderung
  fn send_gl(
    &mut self, adr: usize, drivemode: GLDriveMode, v: usize, v_max: usize, funktionen: u64,
    refresh: bool,
  ) {
    {
      let gl = self.all_gl.get_mut(&adr).unwrap();
      //Speed bezogen auf v_max von Initkommando berechnen
      let speed = (gl.protokoll_speedsteps * v) / v_max;
      //Neuen Zustand speichern
      gl.direction = drivemode;
      gl.speed = speed;
      gl.fnkt = funktionen;
    }
    //Und versenden
    self.send_gl_tel(adr, refresh);
    //Alle Info Clients über neuen Zustand Informieren
    self.send_info_msg(None, adr);
  }

  /// Versenden Telegram einer GL.
  /// # Arguments
  /// * adr - GA Adresse
  /// * refresh - Wenn false: Basistelegram (Fahren) wird immer versendet, zusätzliche Fx Telegramme
  ///             (Protokollabhängig) nur bei Veränderung.
  ///             Wenn true: es wird immer allles versendet (Lok in Refresh Zyklus)
  fn send_gl_tel(&mut self, adr: usize, refresh: bool) {
    let gl = &self.all_gl[&adr];
    //Passendes Protokoll / Version suchen
    let mut protokoll = self
      .all_protokolle
      .get(&gl.protokoll)
      .unwrap()
      .get(gl.protokoll_version.as_str())
      .unwrap()
      .borrow_mut();
    //Basis GL Telegram erzeugen und zum Booster Versenden
    let mut ddl_tel = protokoll.get_gl_new_tel();
    protokoll.get_gl_basis_tel(
      adr,
      gl.direction,
      gl.speed,
      gl.protokoll_speedsteps,
      gl.fnkt,
      &mut ddl_tel,
    );
    //Zusatztelegramm mit weiteren Fx wenn sich diese verändert haben
    protokoll.get_gl_zusatz_tel(adr, refresh, gl.fnkt, &mut ddl_tel);
    drop(protokoll);
    self.send_tel(&mut ddl_tel);
  }
  /// Senden von GL Telegrammen.
  /// Bis "MIN_ANZ_GL_NO_DELAY" Anzahl initalisierter GL's wird mit Wartezeit zwischen Telegrammen in einem Paket
  /// gearbeitet.
  /// Ab dieser Anzahl GL's über Buffer mit einschieben eines anderen Telegramms optimiert.
  /// # Arguments
  /// * ddl_tel - Das Telegramm, das gesendet werden soll.
  fn send_tel(&mut self, ddl_tel: &mut DdlTel) {
    while ddl_tel.daten.len() > 0 {
      //Wann darf das nächste Tel. gesendet werden falls mit Delay gearbeitet wird
      let time_last = Instant::now();
      self.send(self.spidev, ddl_tel);

      //Und jetzt löschen was gesendet wurde
      ddl_tel.daten.remove(0);
      //Direktes weitersenden wenn nicht genügend GL's vorhanden sind oder wenn kein Delay verlangt wird.
      if (ddl_tel.daten.len() > 0)
        && ((self.all_gl.len() < MIN_ANZ_GL_NO_DELAY) || ddl_tel.delay.is_zero())
      {
        let used_delay = Instant::now() - time_last;
        if used_delay < ddl_tel.delay {
          thread::sleep(ddl_tel.delay - used_delay);
        }
      } else {
        //Optimiertes weitersenden über Buffer
        break;
      }
    }
    //Immer aufrufen, auch wenn dieses Telegramm vollständig gesendet wurde um senden eines eventuell
    //noch im Buffe5r befindlichen Telegrammes zu ermöglichen.
    self.send_buffer(ddl_tel);
  }

  /// Verwaltung von Telegrammen die nicht unmittelbar aufeinander folgend gesendet werden dürfen.
  /// z.B. ist 5ms Pause zwischen zwei DCC Telegrammen an die selbe Adresse notwendig.
  /// Um nicht einfach inneffizent 5ms zu warten werden solche Telegramme zurückgehalten und nach
  /// einem anderen Telegramm gesendet.
  /// Folgendes Verhalten:
  /// - Wenn ein Telegramme mit noch nicht gesendetem Inhalt übergeben wurde -> Buffer.
  /// - Wenn bereits eine DDLTel. auf verzögertes Senden wartete, dann wird dieses gesendet.
  /// Damit ergibt sich eine Rekursion wenn 2 verzögerte Telegramme vorhanden sind, diese werden
  /// abwechselnd abgearbeitet bis nur noch eines (oder keines) mehr vorhanden ist.
  /// Es wird als Vereinfachung davon ausgegangen, dass die notwendige Verzögerung (z.B. 5 / 4 ms für DCC)
  /// immer durch ein anderes Telegramm erreicht werden kann.
  /// # Arguments
  /// * ddl_tel - Das Telegramm, das verzögert gesendet werden soll.
  fn send_buffer(&mut self, ddl_tel: &DdlTel) {
    //Zuerst Telegramm aus Buffer holen
    let mut tel_aus_buffer = self.tel_buffer.clone();
    //Dann dieses Telegramm in Buffer wenn es noch ungesendete Teile hat.
    if ddl_tel.daten.len() > 0 {
      self.tel_buffer = Some(ddl_tel.clone());
    } else {
      self.tel_buffer = None;
    }
    //Wenn ein Telegramm im Buffer war, dann wird dieses jetzt gesendet
    if tel_aus_buffer.is_some() {
      self.send_tel(tel_aus_buffer.as_mut().unwrap());
    }
  }
}

impl SRCPDeviceDDL for DdlGL<'_> {
  /// Empfangenes Kommando validieren
  /// Return true wenn Ok.
  /// Sendet die Antwort Message (Ok / Err) an Sender zurück.
  /// # Arguments
  /// * cmd_msg - Empfangenes Kommando
  fn validate_cmd(&self, cmd_msg: &SRCPMessage) -> bool {
    let mut result = false;
    //Für GL wird unterstützt: INIT, SET, GET
    if let SRCPMessageID::Command { msg_type } = cmd_msg.message_id {
      match msg_type {
        SRCPMessageType::INIT => {
          //Format ist INIT <bus> GL <addr> <protocol> <optional further parameters>
          //Für <protocol> wird im Moment unterstützt:
          // M <protocolversion> <decoderspeedsteps> <numberofdecoderfunctions> -> Märklin Motorola
          // N <protocolversion> <decoderspeedsteps> <numberofdecoderfunctions> -> DCC
          // X <protocolversion> <decoderspeedsteps> <numberofdecoderfunctions> <lokUid> <"lokname"> <mfxfunctioncode1> ... <mfxfunctioncode16> -> MFX
          //5 Parameter müssen vorhanden sein: <addr> <protocol> <protocolversion> <decoderspeedsteps> <numberofdecoderfunctions>
          if cmd_msg.parameter.len() >= 5 {
            //Zuerst das Protokoll
            if let Some(protokoll) = DdlProtokolle::from_str(cmd_msg.parameter[1].as_str()) {
              if let Some(protokolle_impl) = self.all_protokolle.get(&protokoll) {
                if let Some(prot_impl) = protokolle_impl.get(cmd_msg.parameter[2].as_str()) {
                  if prot_impl.borrow().uid() && (cmd_msg.parameter.len() < 6) {
                    self
                      .tx
                      .send(SRCPMessage::new_err(cmd_msg, "419", "list too short"))
                      .unwrap();
                  } else {
                    //Adressprüfung
                    if let Ok(adr) = cmd_msg.parameter[0].parse::<usize>() {
                      if (adr > 0) && (adr <= prot_impl.borrow_mut().get_gl_max_adr()) {
                        //Alle weiteren Parameter ausser "lokname" bei MFX müssen Zahlen >=0 sein
                        result = true;
                        for i in 3..cmd_msg.parameter.len() {
                          if (i != 6) && (cmd_msg.parameter[i].parse::<u32>().is_err()) {
                            result = false;
                            self
                              .tx
                              .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
                              .unwrap();
                            break;
                          }
                        }
                        if result {
                          self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
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
        SRCPMessageType::GET => {
          //Format ist GET <bus> GL <addr>
          if self.validate_get_set(cmd_msg, 1) {
            result = true;
          }
        }
        SRCPMessageType::SET => {
          //Format ist SET <bus> GL <addr> <drivemode> <V> <V_max> <f0> . . <fn>
          if self.validate_get_set(cmd_msg, 4) {
            //Jetzt noch <drivemode> <V> <V_max> <f0> . . <fn>
            if (cmd_msg.parameter[1] == "0"
              || cmd_msg.parameter[1] == "1"
              || cmd_msg.parameter[1] == "2")
              && cmd_msg.parameter[2].parse::<u8>().is_ok()
              && cmd_msg.parameter[3].parse::<u8>().is_ok()
            {
              result = true;
              //Wenn Funktionen vorhanden sind, dann müssen die alle 0 oder 1 sein
              if cmd_msg.parameter.len() > 4 {
                for i in 4..cmd_msg.parameter.len() {
                  if (cmd_msg.parameter[i] != "0") && (cmd_msg.parameter[i] != "1") {
                    result = false;
                    self
                      .tx
                      .send(SRCPMessage::new_err(cmd_msg, "412", "wrong value"))
                      .unwrap();
                  }
                }
              }
              if result {
                self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
              }
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
        //Format ist INIT <bus> GL <addr> <protocol> <optional further parameters>
        //Für <protocol> wird im Moment unterstützt:
        // M <protocolversion> <decoderspeedsteps> <numberofdecoderfunctions> -> Märklin Motorola
        // N <protocolversion> <decoderspeedsteps> <numberofdecoderfunctions> -> DCC
        // X <protocolversion> <decoderspeedsteps> <numberofdecoderfunctions> <lokUid> <"lokname"> <mfxfunctioncode1> ... <mfxfunctioncode16> -> MFX
        //Adresse
        let adr = cmd_msg.parameter[0].parse::<usize>().unwrap();
        //Das Protokoll
        let Some(protokoll) = DdlProtokolle::from_str(cmd_msg.parameter[1].as_str()) else {return};
        //Version
        let protokoll_version = cmd_msg.parameter[2].clone();
        //decoderspeedsteps
        let protokoll_speedsteps = cmd_msg.parameter[3].parse::<usize>().unwrap();
        //numberofdecoderfunctions
        let protokoll_number_functions = cmd_msg.parameter[4].parse::<usize>().unwrap();
        //Protokoll spezifisches Init
        {
          //Passendes Protokoll / Version suchen
          let mut protokoll = self
            .all_protokolle
            .get(&protokoll)
            .unwrap()
            .get(protokoll_version.as_str())
            .unwrap()
            .borrow_mut();
          let uid = if protokoll.uid() {
            cmd_msg.parameter[5].parse::<u32>().unwrap()
          } else {
            0
          };
          protokoll.init_gl(adr, uid, protokoll_number_functions);
        }
        self.all_gl.insert(
          adr,
          GLInit::new(
            protokoll,
            protokoll_version,
            protokoll_speedsteps,
            protokoll_number_functions,
          ),
        );
        //INFO <bus> GL <adr> <protokoll> <protocolversion> <decoderspeedsteps> <numberofdecoderfunctions> .....
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
        //OK an diese Session
        self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
        //Das hier verwendete Protokoll ist nicht mehr Idle
        let index_used_prot = self
          .all_idle_protokolle
          .iter()
          .position(|&prot| prot == protokoll);
        if let Some(i) = index_used_prot {
          self.all_idle_protokolle.remove(i);
        }
      }
      SRCPMessageType::GET => {
        //Format ist GET <bus> GL <addr>
        let adr = cmd_msg.parameter[0].parse::<usize>().unwrap();
        //INFO <bus> GL <addr> <drivemode> <V> <V_max> <f0> . . <fn>
        self.send_info_msg(cmd_msg.session_id, adr);
      }
      SRCPMessageType::SET => {
        //Format ist SET <bus> GL <addr> <drivemode> <V> <V_max> <f0> . . <fn>
        let adr = cmd_msg.parameter[0].parse::<usize>().unwrap();
        let drivemode = GLDriveMode::from_str(cmd_msg.parameter[1].as_str()).unwrap();
        let v = cmd_msg.parameter[2].parse::<usize>().unwrap();
        let v_max = cmd_msg.parameter[3].parse::<usize>().unwrap();
        let mut funktionen: u64 = 0;
        if cmd_msg.parameter.len() > 4 {
          for i in 4..cmd_msg.parameter.len() {
            if cmd_msg.parameter[i] == "1" {
              funktionen |= 1 << (i - 4);
            }
          }
        }
        self.send_gl(adr, drivemode, v, v_max, funktionen, false);
        //OK an diese Session
        self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
      }
    };
  }

  /// Refresh Zyklus Telegramm senden (wird nur für GL aufgerufen)
  /// Solange keine GL's vorhanden isnd wird bei jedem Aufruf von jedem vorhandenen Protokoll
  /// das Idle Telegramm gesendet.
  /// Sobald GL's vorhanden sind, wird Zyklisch jede GL wiederholt.
  /// Wenn alle GL durch sind, dann wird non jedem noch unbenutztem Protokoll das Idle Tel. gesendet.
  /// Wenn es keine unbenutzten Protokolle mehr hat, dann wird bei diesem Aufruf nichts mehr gemacht.
  fn send_refresh(&mut self) {
    for (adr, _) in &self.all_gl {
      if self.adr_refresh == 0 {
        //Nächste Refreshadr. gefunden
        self.adr_refresh = *adr;
        break;
      }
      if *adr == self.adr_refresh {
        //Nächste Adresse ist nächste Refreshadr.
        self.adr_refresh = 0;
      }
    }
    //Wenn Refresh Adr. nun 0 ist, dann war das gerade die letzte (Überlauf) oder es gibt noch gar keine GL's.
    //Von allen vorhandenen Protokollen das Idle Telegramm senden, wenn das Protokoll nicht schon gebraucht
    //wurde. Wenn alle Protokolle bereits mit GL verwendet werden, dann machen wir hier einmal nichts, nächster Aufruf kommt wieder.
    if self.adr_refresh == 0 {
      for i in 0..self.all_idle_protokolle.len() {
        //Immer erste vorhandene Version für Idle Tel. verwenden
        let idle_protokoll = self.all_protokolle[&self.all_idle_protokolle[i]]
          .values()
          .next()
          .unwrap();
        let mut idle_tel = idle_protokoll.borrow().get_idle_tel();
        if let Some(tel) = idle_tel.as_mut() {
          self.send_tel(tel);
        }
      }
    } else {
      //Sobald eine Lok vorhanden ist, Refresh senden
      self.send_gl_tel(self.adr_refresh, true);
    }
  }

  /// Alle internen zustände als Info Message versenden
  /// # Arguments
  /// * session_id - SRCP Client Session ID an die die Zustände gesendet werden sollen.
  ///                None -> Info an alle SRCP Clients
  fn send_all_info(&self, session_id: Option<u32>) {
    //Über alle initialisierten GA's
    for (adr, _) in &self.all_gl {
      self.send_info_msg(session_id, *adr);
    }
  }
}
