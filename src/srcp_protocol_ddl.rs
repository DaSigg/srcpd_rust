use std::{cell::RefCell, collections::HashMap, rc::Rc, time::Duration};

//SPI Baudrate für MFX.
//Auch hier müssen wir auf sicher 96 Bytes kommen um im DMA Modus zu sein und keine Pause zwischen den Bytes zu haben.
//1 Bit in MFX sind immer 100us. 1 Bit wird auf ein SPI Byte gelegt, also für ein Bit 100 / 8 = 12.5us -> 80000 Baud
//Grösse Paket wird damit (mit Sync Muster) für das kleinst mögliche Paket, das wir senden:
const _SPI_BAUDRATE_MFX: u32 = 80000;
//Minimales Paket das verwendet wird, ist "Fahren kurz" plus "Funktionen kurz"
// - 0            :  4 Bit -> Sicherstellen, dass Startpegel immer 0 ist
// - Sync         :  5 Bit
// - 10AAAAAAA    :  9 Bit Adresse (Minimal bei 7 Bit Adresse)
// - Kommando     :  7 Bit "Fahren kurz" 3+1+3
// - Kommando     :  7 Bit "Funktionen kurz" 3+4
// - Checksumme   :  8 Bit
// - Sync         : 10 Bit
// - Total        : 50 Bit
//Auch hier kommt nun deshalb wieder die doppelte Baudrate ins Spiel, damit wären wir bei 2*50=100 Bytes.
//1 MFX Bit -> 2 SPI Bytes
//Und mit mehr als 16 Funktionen haben wir ein neues minimales Paket:
// - 0            :  4 Bit -> Sicherstellen, dass Startpegel immer 0 ist
// - Sync         :  5 Bit
// - 10AAAAAAA    :  9 Bit Adresse (Minimal bei 7 Bit Adresse)
// - Kommando     : 12 Bit "Funktion einzeln" 3+7+1+1
// - Checksumme   :  8 Bit
// - Sync         : 10 Bit
// - Total        : 48 Bit
//Und Glück gehabt, wir sind mit mal 2 gerade auf 96 gekommen...
const _SPI_BAUDRATE_MFX_2: u32 = _SPI_BAUDRATE_MFX * 2;

/// Telegramm zum senden über SPI
#[derive(Debug, Clone)]
pub struct DdlTel {
  /// Die Baudrate mit der gesendet werden muss
  pub hz: u32,
  /// Die minimale Verzögerung in ms vom Start eines zum nächsten Telegramm.
  pub delay: Duration,
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
  /// * capacity - Initiale reservierte Grösse für Nutzdaten im ersten erstellten Telegramm
  pub fn new(hz: u32, delay: Duration, capacity: usize) -> DdlTel {
    DdlTel {
      hz,
      delay,
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
  fn get_gl_new_tel(&self) -> DdlTel;
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
  /// * funk_anz - Anzahl tatsächlich verwendete Funktionen. Kann, je nach Protokoll, dazu
  ///              verwendet werden, nur Telegramme der verwendeten Funktionen zu senden.
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_gl_zusatz_tel(
    &mut self, adr: usize, refresh: bool, funktionen: u64, funk_anz: usize, ddl_tel: &mut DdlTel,
  );
  /// Liefert ein leeres GA Telegramm zur Verwendung in "get_ga_tel".
  fn get_ga_new_tel(&self) -> DdlTel;
  /// Erzeugt ein GA Telegramm
  /// # Arguments
  /// * adr - Adresse des Schaltdekoders
  /// * port - Port auf dem Schaltdekoder
  /// * value - Gewünschter Zustand des Port Ein/Aus
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_ga_tel(&self, adr: usize, port: usize, value: bool, ddl_tel: &mut DdlTel);
  /// Liefert das Idle Telegramm dieses Protokolles
  fn get_idle_tel(&self) -> DdlTel;
}

/// Typen zu Verwaltung der Protokolle
pub type HashMapVersion = HashMap<&'static str, Rc<RefCell<dyn DdlProtokoll>>>;
pub type HashMapProtokollVersion = HashMap<DdlProtokolle, HashMapVersion>;
