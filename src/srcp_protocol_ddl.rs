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
  pub adr: usize,
  /// Und auch zum debuggen: Triggerimpuls für Oszi bei senden dieses Telegrammes ausgeben
  pub trigger: bool,
  /// Die Baudrate mit der gesendet werden muss
  pub hz: u32,
  /// Notwendige Pause vor Paket. MUSS zusätzlich gemacht werden
  pub pause_start: Duration,
  /// Notwendige Pause nach Paket. Ist in Daten enthalten, dient hier zur Information
  /// um eine allfällige folgende Startpause optimieren zu können
  pub pause_ende: Duration,
  /// Die minimale Verzögerung in ms vom Start eines zum nächsten Telegramm wenn mehrere Telegramme in einem DdlTel sind.
  pub delay: Duration,
  /// Ab wann darf das nächste Telegramm gesendet werden:
  /// Zeitpunkt Ende Versenden letztes + delay
  pub instant_next: Option<Instant>,
  /// Die Bytes die gesendet werden müssen
  /// Es können hier mehrere Unabhöngige Telegramme zurückgegeben werden wenn diese nicht
  /// unmittelbar nacheinander gesendet werden dürfen. z.B. verlangt DCC 5ms zwischen 2 Telegrammen
  /// an die selbe Adresse, was für Fahren und F0-F5 immer der Fall ist.
  /// Wenn mehr als ein Telegramm zurückgegeben wird, dann erfolgt die Ausgabe immer abwechlungsweise
  /// mit einem Telegramm zu einer anderen Adresse (was auch ein anderes Protokoll sein kein).
  pub daten: Vec<Vec<u8>>,
}
impl DdlTel {
  /// Neue Instanz Erstellen
  /// # Arguments
  /// * hz - Zur Ausgane über SPI notwendige Baurate
  /// * pause_start - Notwendige Pause vor Paket.
  /// * pause_ende - Information Pause nach Paket die bereits in "daten" enthalten ist.
  /// * delay - Die minimale Verzögerung in ms vom Start eines zum nächsten Telegramm wenn in "daten" mehr als ein Telegramm vorhanden ist.
  /// * capacity - Initiale reservierte Grösse für Nutzdaten im ersten erstellten Telegramm
  pub fn new(
    adr: usize, hz: u32, pause_start: Duration, pause_ende: Duration, delay: Duration,
    capacity: usize,
  ) -> DdlTel {
    DdlTel {
      adr,
      trigger: false,
      hz,
      pause_start,
      pause_ende,
      delay,
      instant_next: None,
      daten: vec![Vec::with_capacity(capacity)],
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
  /// * uid - UID des Dekoders
  /// * funk_anz - Anzahl tatsächlich verwendete Funktionen. Kann, je nach Protokoll, dazu
  ///              verwendet werden, nur Telegramme der verwendeten Funktionen zu senden.
  fn init_gl(&mut self, adr: usize, uid: u32, funk_anz: usize);
  /// Liefert die max. erlaubte Lokadresse
  fn get_gl_max_adr(&self) -> usize;
  /// Liefert die max. erlaubte Schaltmoduladdresse
  fn get_ga_max_adr(&self) -> usize;
  /// Liefert die max. Anzahl der unterstützten Funktionen
  fn get_gl_anz_f(&self) -> usize;
  /// Liefert die Anzahl Funktionen (inkl. F0) die im Basistelegramm enthalten sind
  /// Muss immer <= "get_Anz_F" sein.
  fn get_gl_anz_f_basis(&self) -> usize;
  /// Liefert ein leeres GL Telegramm zur Verwendung in "get_gl_basis_tel" und / oder "get_gl_zusatz_tel".
  /// # Arguments
  /// * adr - Adresse der Lok, keine Verwendunbg, nur Debug Support
  fn get_gl_new_tel(&self, adr: usize) -> DdlTel;
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
    &mut self, adr: usize, drive_mode: GLDriveMode, speed: usize, speed_steps: usize,
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
  fn get_gl_zusatz_tel(&mut self, adr: usize, refresh: bool, funktionen: u64, ddl_tel: &mut DdlTel);
  /// Liefert ein leeres GA Telegramm zur Verwendung in "get_ga_tel".
  /// # Arguments
  /// * adr - Adresse GA, keine Verwendunbg, nur Debug Support
  fn get_ga_new_tel(&self, adr: usize) -> DdlTel;
  /// Erzeugt ein GA Telegramm
  /// # Arguments
  /// * adr - Adresse des Schaltdekoders
  /// * port - Port auf dem Schaltdekoder
  /// * value - Gewünschter Zustand des Port Ein/Aus
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_ga_tel(&self, adr: usize, port: usize, value: bool, ddl_tel: &mut DdlTel);
  /// Liefert das Idle Telegramm dieses Protokolles
  /// Return None wenn kein Idle Telegramm vorhanden ist
  fn get_idle_tel(&mut self) -> Option<DdlTel>;
  /// Liefert zusätzliche, Protokoll spzifische Telegramme (z.B. bei MFX die UID & Neuanmeldezähler der Zentrale)
  /// Liefert None, wenn es nichts zur versenden gibt
  fn get_protokoll_telegrammme(&mut self) -> Option<DdlTel> {
    None
  }
}

/// Typen zu Verwaltung der Protokolle
pub type HashMapVersion = HashMap<&'static str, Rc<RefCell<dyn DdlProtokoll>>>;
pub type HashMapProtokollVersion = HashMap<DdlProtokolle, HashMapVersion>;
