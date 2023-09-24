use std::{
  collections::HashMap,
  sync::mpsc::Sender,
  thread,
  time::{Duration, Instant},
};

use log::{info, warn};
use spidev::Spidev;

use crate::{
  srcp_devices_ddl::SRCPDeviceDDL,
  srcp_protocol_ddl::{
    DdlProtokolle, DdlTel, GLDriveMode, HashMapProtokollVersion, ResultReadGlParameter,
  },
  srcp_server_types::{SRCPMessage, SRCPMessageDevice, SRCPMessageID, SRCPMessageType},
};

/// Anzahl initialisierter GL damit in Modus ohne extra Delays ziwschen Telegrammen
/// an die selbe GL  gewechselt wird.
/// Dies muss genügend gross sein, um einen verzögert gesendeten Buffer einer GL innerhalb eines
/// Refreshzyklus abzubauen.
/// Theoretischer Worst Case: DCC Lok mit 64 Funktionen könnte zu 11 Telegrammen führen
const MIN_ANZ_GL_NO_DELAY: usize = 15;
/// Anzahl GL's die MM oder DCC verwenden müssen, damit das Protokoll nicht mehr als Idle gilt.
/// Grund: Bei DCC die Möglichkeit haben 5ms Verzögerungen zu machen, bei MM darf nicht nur eine
/// MM Adresse vorhanden sein wegen Dekoder Prog. Modus.
const IDLE_COUNT_MM_DCC: usize = 2;

///Verwaltung einer initialisierten GL's
#[derive(Clone)]
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
  //Wenn vorhanden: UID der GL
  protokoll_uid: Option<u32>,
  //Optionale Init Parameter (z.B. MFX UID, Name, Funktionen)
  param: Vec<String>,
}
impl GLInit {
  fn new(
    protokoll: DdlProtokolle, protokoll_version: String, protokoll_speedsteps: usize,
    protokoll_number_functions: usize, protokoll_uid: Option<u32>, param: &Vec<String>,
  ) -> GLInit {
    GLInit {
      protokoll,
      protokoll_version,
      protokoll_speedsteps,
      protokoll_number_functions,
      direction: GLDriveMode::Vorwaerts,
      speed: 0,
      fnkt: 0,
      protokoll_uid,
      param: param.clone(),
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
  all_gl: HashMap<u32, GLInit>,
  ///Letzte GL Adr. die im Refreshzyklus war. 0 solange keine GL vorhanden ist.
  adr_refresh: u32,
  ///Alle noch nicht durch GL verwendeten aber vorhandenen Protokolle für Idle Telegramme
  all_idle_protokolle: Vec<DdlProtokolle>,
  ///Buffer für verzögertes senden
  tel_buffer: Vec<DdlTel>,
  ///GL's, die automatisch angemeldet wurden und bei der noch die optionalen Parameter ausgelesen werden
  ///Es wird nur immer eine Lok gleichzeitig angemeldet, wenn eine Lok in SM ist, finden keine Anmeldungen statt.
  gl_param_read: Option<u32>,
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
      tel_buffer: Vec::new(),
      gl_param_read: None,
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
      if let Ok(adr) = cmd_msg.parameter[0].parse::<u32>() {
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
  fn send_info_msg(&self, session_id: Option<u32>, adr: u32) {
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
    &mut self, adr: u32, drivemode: GLDriveMode, v: usize, v_max: usize, funktionen: u64,
    refresh: bool,
  ) {
    let mut doppelt = false;
    {
      let gl = self.all_gl.get_mut(&adr).unwrap();
      //Speed bezogen auf v_max von Initkommando berechnen
      let speed = (gl.protokoll_speedsteps * v) / v_max;
      if (gl.speed > 0) && (speed == 0) {
        //Neu angehalten, zur Sicherheit doppelt senden
        doppelt = true;
      }
      //Neuen Zustand speichern
      gl.direction = drivemode;
      gl.speed = speed;
      gl.fnkt = funktionen;
    }
    //Und versenden
    self.send_gl_tel(adr, doppelt, refresh);
    //Alle Info Clients über neuen Zustand Informieren
    self.send_info_msg(None, adr);
  }

  /// Versenden Telegram einer GL.
  /// # Arguments
  /// * adr - GA Adresse
  /// * doppelt - true: verdoppelte Ausgabe (z.B. wenn Lok neu angehalten wurde)
  /// * refresh - Wenn false: Basistelegram (Fahren) wird immer versendet, zusätzliche Fx Telegramme
  ///             (Protokollabhängig) nur bei Veränderung.
  ///             Wenn true: es wird immer allles versendet (Lok in Refresh Zyklus)
  fn send_gl_tel(&mut self, adr: u32, doppelt: bool, refresh: bool) {
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
    let mut ddl_tel = protokoll.get_gl_new_tel(adr, refresh);
    if doppelt {
      ddl_tel.tel_wiederholungen *= 2;
    }
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
      <DdlGL<'_> as SRCPDeviceDDL>::send(self.spidev, ddl_tel);

      //Direktes weitersenden wenn nicht genügend GL's vorhanden sind oder wenn kein Delay verlangt wird.
      if (ddl_tel.daten.len() > 0)
        && ((self.all_gl.len() < MIN_ANZ_GL_NO_DELAY) || ddl_tel.delay.is_zero())
      {
        if (!ddl_tel.delay.is_zero()) && (ddl_tel.daten.len() > 0) {
          thread::sleep(ddl_tel.delay);
        }
      } else {
        //Wenn ein delay vorhanden ist und dieser nur auf das 2. Telegramm wirken soll, dann kann er jetzt sicher weg
        if ddl_tel.delay_only2nd {
          ddl_tel.delay = Duration::ZERO;
        }
        //Optimiertes weitersenden über Buffer
        break;
      }
    }
    //Wenn noch Telegramme zum verzögert senden vorhanden sind -> in Buffer
    if ddl_tel.daten.len() > 0 {
      self.tel_buffer.push(ddl_tel.clone());
    }
    //Immer aufrufen, auch wenn dieses Telegramm vollständig gesendet wurde um senden eines eventuell
    //noch im Buffer befindlichen Telegrammes zu ermöglichen.
    self.send_buffer();
  }

  /// Senden von Telegrammen die nicht unmittelbar aufeinander folgend gesendet werden dürfen.
  /// z.B. ist 5ms Pause zwischen zwei DCC Telegrammen an die selbe Adresse notwendig,
  /// 50ms bei MM5 zwischen den beiden Telegrammen für 28 v Stufen.
  /// Es wird immer der ganze Buffer abgearbeitet und alles, was möglich ist, gesendet.
  /// Abbruch erfolgt erst dann, wenn Buffer leer ist oder in einem Durchgang gar nichts gesendet werden konnte.
  fn send_buffer(&mut self) {
    let mut done = false;
    while !done {
      done = true;
      for ddl_tel in self.tel_buffer.iter_mut() {
        if ddl_tel.instant_next.unwrap() <= Instant::now() {
          <DdlGL<'_> as SRCPDeviceDDL>::send(self.spidev, ddl_tel);
          done = false;
        }
      }
      if !done {
        //Es wurde etwas gesendet, alle nun leeren Telegramme löschen
        self.tel_buffer.retain(|ddl_tel| !ddl_tel.daten.is_empty());
      }
    }
  }

  /// Ermittlung, durch wieviele GL's ein Protokoll verwendet wird
  /// # Arguments
  /// * protokoll - Das Protokoll, das gesucht werden soll.
  fn count_protokoll(&self, protokoll: DdlProtokolle) -> usize {
    let mut count = 0;
    for (_, gl) in &self.all_gl {
      if gl.protokoll == protokoll {
        count += 1;
      }
    }
    count
  }

  /// Neue GL registrieren
  /// # Arguments
  /// * adr - Adresse der GL.
  /// * protokoll - verwendetes Protokoll
  /// * protokoll_version - verwendete Version des Protokolls
  /// * speedsteps - Anzahl Speedsteps mit denen die GL initialisiert ist
  /// * number_functions - Anzahl verwendete Funktionen
  /// * uid - Dekoder UID wenn vorhanden
  /// * param - Optionale, Protokollabhängige Paramater (z.B. MFX UID, Name, Funktionen)
  fn register_new_gl(
    &mut self, adr: u32, protokoll: &DdlProtokolle, protokoll_version: &str, speedsteps: usize,
    number_functions: usize, uid: Option<u32>, param: &Vec<String>,
  ) -> &GLInit {
    self.all_gl.insert(
      adr,
      GLInit::new(
        *protokoll,
        protokoll_version.to_string(),
        speedsteps,
        number_functions,
        uid,
        param,
      ),
    );
    self.all_gl.get(&adr).unwrap()
  }
  /// Neue GL als SRCP Info melden
  /// # Arguments
  /// * adr - Adresse der GL.
  /// * new_gl - Neue GL die gemeldet werden soll
  fn srcp_info_new_gl(&mut self, adr: u32, new_gl: &GLInit) {
    //INFO <bus> GL <adr> <protokoll> <protocolversion> <decoderspeedsteps> <numberofdecoderfunctions> .....
    //Alles nach GL sind Parameter
    let mut parameter: Vec<String> = vec![];
    parameter.push(adr.to_string());
    parameter.push(new_gl.protokoll.to_string());
    parameter.push(new_gl.protokoll_version.to_string());
    parameter.push(new_gl.protokoll_speedsteps.to_string());
    parameter.push(new_gl.protokoll_number_functions.to_string());
    parameter.extend(new_gl.param.clone());

    self
      .tx
      .send(SRCPMessage::new(
        None,
        self.bus,
        SRCPMessageID::Info {
          info_code: "100".to_string(),
        },
        SRCPMessageDevice::GL,
        parameter,
      ))
      .unwrap();
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
                    if let Ok(adr) = cmd_msg.parameter[0].parse::<u32>() {
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
        SRCPMessageType::TERM => {
          //Format ist TERM <bus> GL <addr>
          //Adressprüfung
          if let Ok(adr) = cmd_msg.parameter[0].parse::<u32>() {
            if self.all_gl.contains_key(&adr) {
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
              && (cmd_msg.parameter[3].parse::<u8>().unwrap() > 0)
            //vmax muss > 0 sein
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
              //OK wird bei SET bereits in Validate gesendet da SET Kommando bei Power Off zuerst in die Queue kommt.
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
        let adr = cmd_msg.parameter[0].parse::<u32>().unwrap();
        //Das Protokoll
        let Some(protokoll) = DdlProtokolle::from_str(cmd_msg.parameter[1].as_str()) else {return};
        //Version
        let protokoll_version = cmd_msg.parameter[2].clone();
        //decoderspeedsteps
        let protokoll_speedsteps = cmd_msg.parameter[3].parse::<usize>().unwrap();
        //numberofdecoderfunctions
        let protokoll_number_functions = cmd_msg.parameter[4].parse::<usize>().unwrap();
        //UID falls vorhanden
        let mut uid = None;
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
          if protokoll.uid() {
            uid = Some(cmd_msg.parameter[5].parse::<u32>().unwrap());
          }
          protokoll.init_gl(adr, uid, protokoll_number_functions);
        }

        let new_gl = self
          .register_new_gl(
            adr,
            &protokoll,
            protokoll_version.as_str(),
            protokoll_speedsteps,
            protokoll_number_functions,
            uid,
            &cmd_msg.parameter[5..].to_vec(), //Alle Paramater ab UID
          )
          .clone();
        self.srcp_info_new_gl(adr, &new_gl);
        //OK an diese Session
        self.tx.send(SRCPMessage::new_ok(cmd_msg, "200")).unwrap();
        //Das hier verwendete Protokoll ist nicht mehr Idle
        let index_used_prot = self
          .all_idle_protokolle
          .iter()
          .position(|&prot| prot == protokoll);
        if let Some(i) = index_used_prot {
          //MM Protokoll wird erst ab 2 GL's aus Idle genommen.
          //Grund: wenn MM Dekoder nur ihre eigene Adresse und gar nichts anderes sehen können sie nach Power Up
          //in den MM Programmiermodus gehen ... :-(
          //Auch DCC wird erst ab 2 GL's aus Idle genommen.
          //Grund: 5ms Verzögerung von einem GL bis zum nächsten mit selber Adresse.
          let mut idle = true;
          if (protokoll == DdlProtokolle::Maerklin) || (protokoll == DdlProtokolle::Dcc) {
            idle = self.count_protokoll(protokoll) >= IDLE_COUNT_MM_DCC;
          }
          if idle {
            self.all_idle_protokolle.remove(i);
          }
        }
      }
      SRCPMessageType::TERM => {
        //Format ist TERM <bus> GL <addr>
        //Adresse
        let adr = cmd_msg.parameter[0].parse::<u32>().unwrap();
        let protokoll = self.all_gl.remove(&adr).unwrap().protokoll;
        //Ein Protokoll könnte wieder Idle geworden sein.
        //Märklin & DCC bei < 2, siehe oben
        let prot_count = self.count_protokoll(protokoll);
        if (prot_count == 0)
          || (((protokoll == DdlProtokolle::Maerklin) || (protokoll == DdlProtokolle::Dcc))
            && (prot_count < IDLE_COUNT_MM_DCC))
        {
          //Es ist Idle
          if !self.all_idle_protokolle.contains(&protokoll) {
            self.all_idle_protokolle.push(protokoll);
          }
        }
      }
      SRCPMessageType::GET => {
        //Format ist GET <bus> GL <addr>
        let adr = cmd_msg.parameter[0].parse::<u32>().unwrap();
        //INFO <bus> GL <addr> <drivemode> <V> <V_max> <f0> . . <fn>
        self.send_info_msg(cmd_msg.session_id, adr);
      }
      SRCPMessageType::SET => {
        //Format ist SET <bus> GL <addr> <drivemode> <V> <V_max> <f0> . . <fn>
        let adr = cmd_msg.parameter[0].parse::<u32>().unwrap();
        //Da SET verzögert über Queue ausgeführt wird könnte ein TERM dazwischen gekommen sein, Adresse nochmals prüfen
        if self.all_gl.contains_key(&adr) {
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
          //OK an diese Session wurde bei Validate bereits gesendet da SET ohne POWER zuerst in Queue kommt.
        }
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
        let mut idle_tel = idle_protokoll.borrow_mut().get_idle_tel();
        if let Some(tel) = idle_tel.as_mut() {
          self.send_tel(tel);
        }
      }
    } else {
      //Sobald eine Lok vorhanden ist, Refresh senden
      self.send_gl_tel(self.adr_refresh, false, true);
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
  /// Muss zyklisch aufgerufen werden. Erlaubt dem Device die Ausführung von
  /// von neuen Kommando oder refresh unabhängigen Aufgaben.
  /// Wird hier verwendet um allen vorhandenen Protokollen die Möglichkeit zu geben
  /// ihr periodische Aufgaben / Telegramme auszuführen.
  /// # Arguments
  /// * power - true: Power / Booster ist ein, Strom auf den Schienen
  ///           false: Power / Booster ist aus
  fn execute(&mut self, power: bool) {
    //Ohne Power macht es auch keinen Sinn Telegramme zu senden
    if power {
      for (_protokoll, prot_versionen) in &self.all_protokolle.clone() {
        for (_version, prot_impl) in prot_versionen {
          let mut p = prot_impl.borrow_mut();
          //SM (keine Loksuche) wenn eine GL in SM ist oder bereits eine auto Anmeldung läuft
          if let Some(tel) = p.get_protokoll_telegrammme().as_mut() {
            self.send_tel(tel);
            //Wenn verlangt wurde, dass ein Ergebnis eingelesen wird -> Auswerten
            if let Some(daten_rx) = &tel.daten_rx {
              if let Some(uid) = p.eval_neu_anmeldung(daten_rx) {
                //Noch nicht angemeldeter Dekoder gefunden.
                //Wenn es die GL mit dieser UID schon gibt, dann wird dessen Adressen verwendet.
                let mut gl_bekannt = false;
                for adr in 1..=p.get_gl_max_adr() {
                  if let Some(gl) = self.all_gl.get(&adr) {
                    if gl.protokoll_uid.is_some() && (gl.protokoll_uid.unwrap() == uid) {
                      //Lok gibt es bereits, neue SID Zuordnung auslösen
                      info!("GL: bekannte Lok gefunden UID={}, Adr={}", uid, adr);
                      //Freie Adresse gefunden, Protokollabhängige Aktionen wie SID Zuordnung versenden auslösen
                      p.init_gl(adr, gl.protokoll_uid, gl.protokoll_number_functions);
                      gl_bekannt = true;
                      break;
                    }
                  }
                }
                //Ansonsten die erste freie GL Adresse zuweisen und Initialisieren.
                if !gl_bekannt {
                  for adr in 1..=p.get_gl_max_adr() {
                    if !self.all_gl.contains_key(&adr) {
                      info!("GL: neue Lok gefunden UID={}, Adr={}", uid, adr);
                      //Es werden mal die im Basistel. enthalten Funktionen als vorhanden angenommen (bei MFX 16).
                      let anz_f = p.get_gl_anz_f_basis();
                      //Freie Adresse gefunden, Protokollabhängige Aktionen wie SID Zuordnung versenden auslösen
                      p.init_gl(adr, Some(uid), anz_f);
                      //Neue GL ist mal angemeldet, kann prinzipiell verwendet werden.
                      //Bevor sie über SRCP INFO gemeldet wird, wird noch versucht optionale Parameter auszulesen.
                      self.gl_param_read = Some(adr);
                      break;
                    }
                  }
                }
              }
            }
          }
          if self.gl_param_read.is_none() {
            //Wenn keine Lokanmeldung aktiv ist, dann könnte ein SM Meldung bereit liegen
            todo!()
          }
        }
      }
      //Optionale GL Parameter für automatisch neu angemeldete GL's lesen
      if let Some(adr) = self.gl_param_read {
        let mut send_info = false;
        //Falls es die GL in der Zwischenzeit nicht mehr gibt
        if let Some(gl) = self.all_gl.get_mut(&adr) {
          //Passendes Protokoll / Version suchen
          let mut protokoll = self
            .all_protokolle
            .get(&gl.protokoll)
            .unwrap()
            .get(gl.protokoll_version.as_str())
            .unwrap()
            .borrow_mut();
          match protokoll.read_gl_parameter(adr) {
            ResultReadGlParameter::Busy => (), //In Arbeit, weiter machen
            ResultReadGlParameter::Error => {
              warn!(
                "GL Lokparameter können nicht gelesen werden für Adr {}",
                adr
              );
              //Neue GL über SRCP Info ohne optionale Parameter melden
              send_info = true;
              self.gl_param_read = None;
            }
            ResultReadGlParameter::Ok(param) => {
              //Ausgelesene Parameter in GL speichern
              gl.param.extend(param);
              //Vollständige SRCP Info Meldung
              send_info = true;
              self.gl_param_read = None;
            }
          }
        } else {
          //GL gibt es nicht mehr, kann hier auch weg.
          self.gl_param_read = None;
        }
        if send_info {
          let gl = self.all_gl[&adr].clone();
          self.srcp_info_new_gl(adr, &gl);
        }
      }
    }
  }
}
