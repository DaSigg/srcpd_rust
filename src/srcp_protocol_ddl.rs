use std::{
  cell::RefCell,
  collections::HashMap,
  rc::Rc,
  time::{Duration, Instant},
};

/// Telegramm zum senden über SPI
#[derive(Debug, Clone)]
pub struct DdlTel {
  /// Nur zu Debugzwecken: Adresse (GL oder GA)
  pub adr: u32,
  /// Und auch zum debuggen: Triggerimpuls für Oszi bei senden dieses Telegrammes ausgeben
  pub trigger: bool,
  /// Wieviel mal wird ein Telegramm direkt hintereinander versendet.
  /// Bei neuen Kommandos >1 (typisch 2), bei Refresh Cycle einmal.
  pub tel_wiederholungen: usize,
  /// Die Baudrate mit der gesendet werden muss
  pub hz: u32,
  /// Die minimale Verzögerung in ms vom Start eines zum nächsten Telegramm wenn mehrere Telegramme in einem DdlTel sind.
  pub delay: Duration,
  /// Wenn delay gesetzt ist und mehr als 2 Telegramme vorhanden sind:
  /// wenn true wirkt der delay nur für 2. Telegramm, für alle andern nicht mehr (MM5), wenn false bei allen (DCC).
  pub delay_only2nd: bool,
  /// Ab wann darf das nächste Telegramm gesendet werden:
  /// Zeitpunkt Ende Versenden letztes + delay
  pub instant_next: Option<Instant>,
  /// Die Bytes die gesendet werden müssen
  /// Es können hier mehrere unabhängige Telegramme zurückgegeben werden. Wenn diese nicht
  /// unmittelbar nacheinander gesendet werden dürfen. z.B. verlangt DCC 5ms zwischen 2 Telegrammen
  /// an die selbe Adresse, was für Fahren und F0-F5 immer der Fall ist.
  /// Wenn mehr als ein Telegramm zurückgegeben wird und ein Delay verlangt ist, dann erfolgt die Ausgabe immer abwechlungsweise
  /// mit einem Telegramm zu einer anderen Adresse (was auch ein anderes Protokoll sein kann).
  pub daten: Vec<Vec<u8>>,
  /// Über SPI eingelesene Daten. Normalerweise nicht verwendete, None.
  /// Wird bei MFX verwendet, die die Erkennung des RDS Signals über SPI IN erfolgt.
  /// Wenn verwendet sollte das Telegramm nur einmal gesendet werden.
  /// Ansonsten sind hier nur die mit der letzten Wiederholung empfangenen Daten enthalten.
  /// Wenn verwendet, dann wird es nur für das letzte Telegramm in "daten" angewandt und die Grösse hier muss genau gleich wie dieses
  /// letzte Telegramm sein.
  pub daten_rx: Option<Vec<u8>>,
}
impl DdlTel {
  /// Neue Instanz Erstellen
  /// # Arguments
  /// * adr - GL/GA Adr zu der dieses Tel. gehört. Nur für Debuging relevant.
  /// * hz - Zur Ausgane über SPI notwendige Baurate
  /// * delay - Die minimale Verzögerung in ms vom Start eines zum nächsten Telegramm wenn in "daten" mehr als ein Telegramm vorhanden ist.
  /// * delay_only2nd - Wenn mehr als zwei Telegramme und ein "delay" vorhanden sind ist bei true der "delay" nur für 2. Telegramm relevant.
  /// * capacity - Initiale reservierte Grösse für Nutzdaten im ersten erstellten Telegramm
  /// * telWiederholungen - Anzahl Wiederholungen beim Senden des Telegrammes
  pub fn new(
    adr: u32, hz: u32, delay: Duration, delay_only2nd: bool, capacity: usize,
    tel_wiederholungen: usize,
  ) -> DdlTel {
    DdlTel {
      adr,
      trigger: false,
      tel_wiederholungen,
      hz,
      delay,
      delay_only2nd,
      instant_next: None,
      daten: vec![Vec::with_capacity(capacity)],
      daten_rx: None,
    }
  }
}

/// Vorhandele Protokolle
#[derive(Clone, Debug, Eq, Hash, PartialEq, Copy)]
pub enum DdlProtokolle {
  //Märklin Motorola I & II
  Maerklin,
  //NMRA DCC
  Dcc,
  //MFX
  Mfx,
}
impl DdlProtokolle {
  pub fn from_str(str: &str) -> Option<DdlProtokolle> {
    match str {
      "M" => Some(DdlProtokolle::Maerklin),
      "N" => Some(DdlProtokolle::Dcc),
      "X" => Some(DdlProtokolle::Mfx),
      _ => None,
    }
  }
}
impl ToString for DdlProtokolle {
  fn to_string(&self) -> String {
    match self {
      DdlProtokolle::Dcc => "N",
      DdlProtokolle::Maerklin => "M",
      DdlProtokolle::Mfx => "X",
    }
    .to_string()
  }
}

/// Lok Richtung
#[derive(Clone, Debug, PartialEq, Copy)]
pub enum GLDriveMode {
  Vorwaerts,
  Rueckwaerts,
  Nothalt,
}
impl GLDriveMode {
  pub fn from_str(str: &str) -> Option<GLDriveMode> {
    match str {
      "0" => Some(GLDriveMode::Rueckwaerts),
      "1" => Some(GLDriveMode::Vorwaerts),
      "2" => Some(GLDriveMode::Nothalt),
      _ => None,
    }
  }
}
impl ToString for GLDriveMode {
  fn to_string(&self) -> String {
    match self {
      GLDriveMode::Rueckwaerts => "0",
      GLDriveMode::Vorwaerts => "1",
      GLDriveMode::Nothalt => "2",
    }
    .to_string()
  }
}

/// Ergebnis für "read_gl_parameter"
pub enum ResultReadGlParameter {
  Error,
  Busy,
  Ok(Vec<String>),
}

/// Schnittstelle für alle Protokolle
/// Wenn mehrere Versionen eines Protokolles vorhanden sind, dann muss dies bei
/// der Implementierung berücksichtigt werden, schlussendlich eine Instanz pro
/// Version erzeugt werden.
pub trait DdlProtokoll {
  /// Legt fest, ob das Protokoll eine UID benötigt, die bei GL INIT Kommando angegeben werden muss
  /// Return true wenn UID benötigt.
  fn uid(&self) -> bool {
    false
  }
  /// GL Init Daten setzen. Welche Daten verwendet werden ist Protokollabhängig.
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * uid - UID des Dekoders, wenn vorhanden
  /// * funk_anz - Anzahl tatsächlich verwendete Funktionen. Kann, je nach Protokoll, dazu
  ///              verwendet werden, nur Telegramme der verwendeten Funktionen zu senden.
  fn init_gl(&mut self, adr: u32, uid: Option<u32>, funk_anz: usize);
  /// Liefert die max. erlaubte Lokadresse
  fn get_gl_max_adr(&self) -> u32;
  /// Wieviele Speedsteps werden vom Protokoll unterstützt
  fn get_gl_max_speed_steps(&self) -> usize;
  /// Liefert die max. erlaubte Schaltmoduladdresse
  fn get_ga_max_adr(&self) -> u32;
  /// Liefert die max. Anzahl der unterstützten Funktionen
  fn get_gl_anz_f(&self) -> usize;
  /// Liefert die Anzahl Funktionen (inkl. F0) die im Basistelegramm enthalten sind
  /// Muss immer <= "get_Anz_F" sein.
  fn get_gl_anz_f_basis(&self) -> usize;
  /// Liefert ein leeres GL Telegramm zur Verwendung in "get_gl_basis_tel" und / oder "get_gl_zusatz_tel".
  /// # Arguments
  /// * adr - Adresse der Lok, keine Verwendunbg, nur Debug Support
  /// * refresh - Wenn true: Aufruf aus Refres Cycle, einmalige Telegramm Versendung,
  ///             Wenn false: Aufruf wegen neuem Lokkommando, mehrmaliges Versenden
  fn get_gl_new_tel(&mut self, adr: u32, refresh: bool) -> DdlTel;
  /// Erzeugt das Basis Telegramm für GL.
  /// - Fahren
  /// - Basisfunktionen F0 bis "get_Anz_F_Basis". Es wedren hier nur diese Funktionen übernommen!
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * drive_mode - Fahrtrichtung / Nothalt
  /// * speed - aktuelle Geschwindigkeit
  /// * speed_steps - Anzahl Speed Steps die verwendet werden soll. Protokoll abhängig.
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt bis "get_Anz_F_Basis"
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_gl_basis_tel(
    &mut self, adr: u32, drive_mode: GLDriveMode, speed: usize, speed_steps: usize,
    funktionen: u64, ddl_tel: &mut DdlTel,
  );
  /// Erzeugt das / die Fx Zusatztelegramm(e) für GL.
  /// - Funktionen nach "get_Anz_F_Basis"
  /// Alle notwendigen Telegramme werden einzeln zu "ddl_tel" hinzugefügt.
  /// Wenn keine extra Telegramme für Zusatzfunktionen vorhanden sind, dann wird nichts hinzugefügt.
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * refresh - Wenn false werden nur Telegramme für Funktionen, die geändert haben, erzeugt
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt ab "get_Anz_F_Basis"
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_gl_zusatz_tel(&mut self, adr: u32, refresh: bool, funktionen: u64, ddl_tel: &mut DdlTel);
  /// Liefert ein leeres GA Telegramm zur Verwendung in "get_ga_tel".
  /// # Arguments
  /// * adr - Adresse GA, keine Verwendunbg, nur Debug Support
  fn get_ga_new_tel(&self, adr: u32) -> DdlTel;
  /// Erzeugt ein GA Telegramm
  /// # Arguments
  /// * adr - Adresse des Schaltdekoders
  /// * port - Port auf dem Schaltdekoder
  /// * value - Gewünschter Zustand des Port Ein/Aus
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_ga_tel(&self, adr: u32, port: usize, value: bool, ddl_tel: &mut DdlTel);
  /// Liefert das Idle Telegramm dieses Protokolles
  /// Return None wenn kein Idle Telegramm vorhanden ist
  fn get_idle_tel(&mut self) -> Option<DdlTel>;
  /// Liefert zusätzliche, Protokoll spezifische Telegramme (z.B. bei MFX die UID & Neuanmeldezähler der Zentrale)
  /// Liefert None, wenn es nichts zur versenden gibt
  fn get_protokoll_telegrammme(&mut self) -> Option<DdlTel> {
    None
  }
  /// Auswertung automatische Dekoderanmeldung (z.B. bei MFX).
  /// Notwendige Telegramme zur Suche müssen über "get_protokoll_telegrammme" ausgegeben und eine Rückmeldung
  /// verlangt werden.
  /// Wenn ein neuer Dekoder gefunden wurde, dann wird dessen UID zurückgegeben, ansonsten None.
  /// # Arguments
  /// * daten_rx : Die beim parallel zum Senden über SPI eingelesenen Daten
  fn eval_neu_anmeldung(&mut self, _daten_rx: &Vec<u8>) -> Option<u32> {
    None
  }
  /// Auslesen optionale GL Parameter (z.B. MFX Lokname und Funktionen)
  /// Liefert ResultReadGlParameter::Error zurück, wenn ein Fehler aufgetreten ist oder vom Protokoll nicht untertsützt.
  /// Liefert ResultReadGlParameter::Busy zurück, wenn das Auslesen im Gange ist
  /// Liefert ResultReadGlParameter::Ok mit den Parametern zurück, wenn abgeschlossen.
  /// Falls "eval_neu_anmeldung" "Some" liefert, sollte hier nicht Error zurückgegeben werden.
  /// Falls vom Protokoll nicht unterstützt, dann Ok mit leerer Liste.
  /// # Arguments
  /// * adr : Schienenadresse der GL
  fn read_gl_parameter(&mut self, _adr: u32) -> ResultReadGlParameter {
    ResultReadGlParameter::Error
  }
  /// Dekoderkonfiguration (SM) Start
  fn sm_init(&mut self) {}
  /// Dekoderkonfiguration (SM) Ende
  fn sm_term(&mut self) {}
  /// Dekoderkonfiguration (SM) Write Value.
  /// # Arguments
  /// * adr - Schienenadresse der GL, 0 für Broadcast
  /// * sm_type - Type des Zugriffes (aus srcp Protokoll)
  /// * para - Parameter für Write Zugriff (protokollabhängig)
  /// * value - zu schreibender Wert
  /// * session_id - Session ID von der das Kommando kam um eine Antwort an diese zu senden.
  fn sm_write(
    &mut self, _adr: u32, _sm_type: &String, _para: &Vec<u32>, _value: u32, _session_id: u32,
  ) {
  }
  /// Dekoderkonfiguration (SM) Read Value.
  /// # Arguments
  /// * adr - Schienenadresse der GL, 0 für Broadcast
  /// * sm_type - Type des Zugriffes (aus srcp Protokoll)
  /// * para - Parameter für Write Zugriff (protokollabhängig)
  /// * session_id - Session ID von der das Kommando kam um eine Antwort an diese zu senden.
  fn sm_read(&mut self, _adr: u32, _sm_type: &String, _para: &Vec<u32>, _session_id: u32) {}
  /// Liefert alle in "sm_read" und "sm_write" unterstützten Typen mit der Anzahl erwarteter Parameter
  /// ohne Value für SET.
  /// None wenn SM nicht unterstützt wird.
  fn sm_get_all_types(&self) -> Option<HashMap<String, usize>> {
    None
  }
}

/// Typen zu Verwaltung der Protokolle
pub type HashMapVersion = HashMap<&'static str, Rc<RefCell<dyn DdlProtokoll>>>;
pub type HashMapProtokollVersion = HashMap<DdlProtokolle, HashMapVersion>;
