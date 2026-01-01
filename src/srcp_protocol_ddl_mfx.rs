use std::{
  collections::HashMap,
  fs,
  sync::mpsc::{self, Receiver, Sender},
  thread,
  time::{Duration, Instant},
};

use log::{info, warn};

use crate::{
  srcp_mfx_rds::{MfxCvTel, MfxCvTelType, MfxRdsFeedbackThread, MfxRdsJob},
  srcp_protocol_ddl::{
    DdlProtokoll, DdlTel, GLDriveMode, ResultNeuAnmeldung, ResultReadGlParameter, SmReadWrite,
  },
};

//SPI Baudrate für MFX.
//Auch hier müssen wir auf sicher 96 Bytes kommen um im DMA Modus zu sein und keine Pause zwischen den Bytes zu haben.
//1 Bit in MFX sind immer 100us. 1 Bit wird auf ein SPI Byte gelegt, also für ein Bit 100 / 8 = 12.5us -> 80000 Baud
//Grösse Paket wird damit (mit Sync Muster) für das kleinst mögliche Paket, das wir senden:
//Hinweis: kleinst möglicher SPI Clock bei 250 MHz core_freq ist 30.5 kHz.
const SPI_BAUDRATE_MFX: u32 = 80000;
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
const SPI_BYTES_PRO_BIT: usize = 2;
const SPI_BAUDRATE_MFX_2: u32 = SPI_BAUDRATE_MFX * (SPI_BYTES_PRO_BIT as u32);

/// Max. erlaubte GL Adresse (14 Bit)
const MAX_MFX_GL_ADRESSE: u32 = 2_u32.pow(14) - 1;

/// Pause vor und nach MFX Paket (Erfahrung aus MFX im orginal srcpd)
const MFX_STARTSTOP_0_BIT: usize = 4;
/// Max. Länge MFX Paket, mit SPI_BAUDRATE_MFX_2 braucht ein Bit immer 2 Byte
/// Entweder
///   Adresse 14 Bit (18 Bit)
///   Fahren 7 Bit (11 Bit)
///   Funktionen F0-15 (20 Bit)
///   Total 49 Bit
/// oder
///   Kommando Znetrale (6 Bit)
///   UID Zentrale (32 Bit)
///   Neuanmeldezähler (16 Bit)
///   Total 54 Bit
/// oder
///   Kommando Schienenadresse (6 Bit)
///   Schienenadresse (14 Bit)
///   UID (32 Bit)
///   Total 52 Bit
/// 3 Syncs am Schluss (Erfahrung aus MFX im orginal srcpd)
/// 8 Bit Pause am Anfang und Schluss (Erfahrung aus MFX im orginal srcpd)
const MFX_MAX_LEN: usize =
  (MFX_STARTSTOP_0_BIT + 5 + 54 + 8 + 15 + MFX_STARTSTOP_0_BIT) * SPI_BYTES_PRO_BIT;
/// 6.4ms Pause für 1 Bit RDS Rückmeldung in Anzahl SPI Bytes, Rechnung mit us, 8Bit=1Byte
const MFX_LEN_PAUSE_6_4_MS: usize = 6400 * SPI_BAUDRATE_MFX_2 as usize / (1000000 * 8);
/// Anzahl Bits für eine RDS Rückmeldeperiode Periode berechnen (+1 um Integer Abrundung der Division aufzurunden)
const BITS_PER_RDSSYNC_PERIOD: usize = 912 * SPI_BAUDRATE_MFX_2 as usize / 1000000 + 1;
const BITS_PER_RDSSYNC_IMP: usize = 25 * SPI_BAUDRATE_MFX_2 as usize / 1000000;
const BITS_PER_RDSSYNC_PAUSE_PERIOD: usize = BITS_PER_RDSSYNC_PERIOD - BITS_PER_RDSSYNC_IMP;
const BITS_PER_RDSSYNC_PAUSE_HALF: usize = (BITS_PER_RDSSYNC_PERIOD / 2) - BITS_PER_RDSSYNC_IMP;
/// Länge Tel. Suchen neuer Dekoder mit 1 Bit RDS Rückmeldung
///   Adresse (7 Bit)
///   Kommando (6 Bit)
///   Anzahl Bits (6 Bit)
///   UID (32 Bit)
///   CRC (8 Bit)
///   11.5 Sync
///   Bitfolge "0011"
///   6.4ms Pause
///   Sync
///   6.4ms Pause
///   2 Sync
const MFX_MAX_LEN_SEARCH_NEW: usize = (MFX_STARTSTOP_0_BIT
  + 5
  + 7
  + 6
  + 6
  + 32
  + 8
  + (12 * 5)
  + 4
  + MFX_LEN_PAUSE_6_4_MS
  + 5
  + MFX_LEN_PAUSE_6_4_MS
  + (2 * 5)
  + MFX_STARTSTOP_0_BIT)
  * SPI_BYTES_PRO_BIT;

/// Alle MFX Kommandos im Format (Value, AnzBits)
type MfxBits = (u32, usize);
/// Adresse 7 Bit
const MFX_ADR_7_BIT: MfxBits = (0b10, 2);
/// Adresse 9 Bit
const MFX_ADR_9_BIT: MfxBits = (0b110, 3);
/// Adresse 11 Bit
const MFX_ADR_11_BIT: MfxBits = (0b1110, 4);
/// Adresse 14 Bit
const MFX_ADR_14_BIT: MfxBits = (0b1111, 4);
/// Kommando Fahren Kurz
const MFX_CMD_FAHREN_KURZ: MfxBits = (0b000, 3);
/// Kommando Fahren
const MFX_CMD_FAHREN: MfxBits = (0b001, 3);
/// Kommando Funktionen F0-F3
const MFX_CMD_FNKT_F0_F3: MfxBits = (0b010, 3);
/// Kommando Funktionen F0-F7
const MFX_CMD_FNKT_F0_F7: MfxBits = (0b0110, 4);
/// Kommando Funktionen F0-F15
const MFX_CMD_FNKT_F0_F15: MfxBits = (0b0111, 4);
/// Kommando Funktionen einzeln
const MFX_CMD_FNKT_EINZELN: MfxBits = (0b100, 3);
/// Kommando Read CV
const MFX_CMD_CV_READ: MfxBits = (0b111000, 6);
/// Kommando Write CV
const MFX_CMD_CV_WRITE: MfxBits = (0b111001, 6);
/// Kommando Suchen neuer Dekoder
const MFX_CMD_SEARCH_NEW: MfxBits = (0b111010, 6);
/// Kommando Konfiguration Schienenadresse
const MFX_CMD_KONFIG_SID: MfxBits = (0b111011, 6);
/// Kommando UID und Neuanmeldezähler Zentrale
const MFX_CMD_KONFIG_UID: MfxBits = (0b111101, 6);

/// Intervall versenden UID Zentrale
const INTERVALL_UID: Duration = Duration::from_millis(500);

pub enum MfxVersion {
  V0, //Analog Implementierung im alten C srcpd
}

pub struct MfxProtokoll {
  /// Version, aktuell nur 0, keine Verwendung.
  _version: MfxVersion,
  /// UID der Zentrale
  uid_zentrale: u32,
  /// Neuanmeldezähler
  reg_counter: u16,
  /// Pfad zum File zur Speicherung Neuanmeldezähler
  path_reg_counter_file: String,
  /// Halten Richtung bei Richtung Nothalt
  old_drive_mode: [GLDriveMode; MAX_MFX_GL_ADRESSE as usize + 1],
  /// Erkennung Funktionswechsel für die nicht immer gesendeten höheren Fx
  old_funktionen: [u64; MAX_MFX_GL_ADRESSE as usize + 1],
  /// Dekoder UID's
  uid: [u32; MAX_MFX_GL_ADRESSE as usize + 1],
  /// Anzahl Initialisierte Funktionen
  funk_anz: [usize; MAX_MFX_GL_ADRESSE as usize + 1],
  /// Muss neue Schienenadr. Zuordung gesendet werden?
  new_sid: [bool; MAX_MFX_GL_ADRESSE as usize + 1],
  /// Anzahl aufeinander folgende 1er für MFX Bitstuffing. Wird mit "add_start_sync" auf 0 gesetzt.
  anz_eins: usize,
  /// Zeitpunkt letztes Versenden UID Zentrale
  zeitpunkt_uid: Instant,
  /// Aktueller Zustand Suche neuer Dekoder
  /// Anzahl gefundene Bits
  search_new_dekoder_bits: u32,
  /// Anzahl gefundene UID
  search_new_dekoder_uid: u32,
  /// Anfangts und Endpositions 1 Bit Rückmeldung im SPI Bytebuffer
  rds_1_bit_start_pos: usize,
  /// Channel für Aufträge an RDS Thread
  tx_to_rds: Sender<MfxRdsJob>,
  /// Channel für Antworten von RDS Thread von SM Read/Write
  rx_from_rds_read_write_ca: Receiver<SmReadWrite>,
  /// Channel für Antworten von RDS Thread von neu angemeldeten GL Init Parametern
  rx_from_rds_lok_init: Receiver<Option<Vec<String>>>,
  /// Channel für Tel. Sendeaufträge vom RDS Thread
  rx_tel_from_rds: Receiver<MfxCvTel>,
  /// Wenn das lesen von Lokparametern im Gange ist, ist hier die Adresse dieser Lok enthalten
  read_gl_parameter: Option<u32>,
  /// Ist SM Mode auf diesem Protokoll aktiviert?
  sm_aktiv: bool,
}

impl MfxProtokoll {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * version - V0
  /// * uid_zentrale - Zu verwendende UID der Zentrale
  /// * path_reg_counter_file - File in dem der Neuanmeldezähler gespeichert ist
  pub fn from(
    version: MfxVersion, uid_zentrale: u32, path_reg_counter_file: String,
  ) -> MfxProtokoll {
    //Neuanmeldezähler laden
    let mut reg_counter: u16 = 0;
    if let Ok(reg_count_str) = fs::read_to_string(&path_reg_counter_file) {
      if let Ok(value) = reg_count_str.parse::<u16>() {
        reg_counter = value;
      }
    } else {
      warn!(
        "MfxProtokoll Neuanmeldezählerfile {path_reg_counter_file} konnte nicht gelesen werden."
      )
    }
    info!("MfxProtokoll Start mit Neuanmeldezähler={reg_counter}");
    //Channels zur Kommunikation mit RDS Thread
    //-> Aufträge zum RDS Thread
    let (tx_to_rds, rx_in_rds): (Sender<MfxRdsJob>, Receiver<MfxRdsJob>) = mpsc::channel();
    //<- Antworten Lok Init vom RDS Thread
    let (tx_from_rds_lok_init, rx_from_rds_lok_init): (
      Sender<Option<Vec<String>>>,
      Receiver<Option<Vec<String>>>,
    ) = mpsc::channel();
    //<- Antworten ReadCA/WriteCA vom RDS Thread
    let (tx_from_rds_read_write_ca, rx_from_rds_read_write_ca): (
      Sender<SmReadWrite>,
      Receiver<SmReadWrite>,
    ) = mpsc::channel();
    //<- MFX Tel. Sendeaufträge vom RDS Thread
    let (tx_tel_from_rds, rx_tel_from_rds): (Sender<MfxCvTel>, Receiver<MfxCvTel>) =
      mpsc::channel();
    //RDS Einlesethread starten
    thread::Builder::new()
      .name("MFX RDS Feedbackthread".to_string())
      .spawn(move || {
        MfxRdsFeedbackThread::new(
          rx_in_rds,
          tx_from_rds_read_write_ca,
          tx_from_rds_lok_init,
          tx_tel_from_rds,
        )
        .execute()
      })
      .unwrap();

    MfxProtokoll {
      _version: version,
      uid_zentrale,
      reg_counter,
      path_reg_counter_file,
      old_drive_mode: [GLDriveMode::Vorwaerts; MAX_MFX_GL_ADRESSE as usize + 1],
      old_funktionen: [0; MAX_MFX_GL_ADRESSE as usize + 1],
      uid: [0; MAX_MFX_GL_ADRESSE as usize + 1],
      funk_anz: [0; MAX_MFX_GL_ADRESSE as usize + 1],
      new_sid: [false; MAX_MFX_GL_ADRESSE as usize + 1],
      anz_eins: 0,
      zeitpunkt_uid: Instant::now(),
      search_new_dekoder_bits: 0,
      search_new_dekoder_uid: 0,
      rds_1_bit_start_pos: 0,
      tx_to_rds,
      rx_from_rds_read_write_ca,
      rx_from_rds_lok_init,
      rx_tel_from_rds,
      read_gl_parameter: None,
      sm_aktiv: false,
    }
  }

  /// Speichern des Neuanmeldezählers
  fn save_registration_counter(&self) {
    if fs::write(&self.path_reg_counter_file, self.reg_counter.to_string()).is_err() {
      warn!("MFX Neuanmeldezähler konnte nicht gespeichert werden.");
    }
  }

  /// Berechnet den MFX CRC.
  /// # Arguments
  /// bits - Die (neuen) Bits
  /// crc - Aktueller CRC, resp Startwert, neuer CRC zurück
  fn crc(&self, bits: MfxBits, crc: &mut u8) {
    let mut crc_long = *crc as u32; // nicht nur 8 Bit, um Carry zu erhalten
    for i in (0..bits.1).rev() {
      //MSB zuerst
      crc_long = (crc_long << 1) | ((bits.0 >> i) & 0x01);
      if (crc_long & 0x0100) > 0 {
        crc_long = (crc_long & 0x00FF) ^ 0x07;
      }
    }
    *crc = crc_long as u8;
  }

  /// Fügt einem MFX Telegramm ein Bit hinzu.
  /// Bit Stuffing Regel wird berücksichtigt.
  /// # Arguments
  /// * bit - true / false
  /// * ddl_tel - Telegramm, bei dem Bits hinzugefügt werden sollen
  fn add_bit(&mut self, bit: bool, ddl_tel: &mut DdlTel) {
    let last = ddl_tel.daten.len() - 1;
    let daten: &mut Vec<u8> = ddl_tel.daten[last].as_mut();
    let values: (u8, u8) = if (*daten.last().unwrap() & 0x01) == 0 {
      (0x00, 0xFF) //Letzter Pegel war 0 -> "normal"
    } else {
      (0xFF, 0x00) //Letzter Pegel war 1 -> invertiert arbeiten
    };
    //Jedes Bit startet mit einer Flanke
    daten.push(values.1);
    //Nun Flankenwechsel für eine 1, keiner für eine 0
    daten.push(if bit { values.0 } else { values.1 });
    //Bitstuffing
    if bit {
      self.anz_eins += 1;
      if self.anz_eins >= 8 {
        //Zwingend eine 0 einschieben
        self.add_bit(false, ddl_tel);
      }
    } else {
      self.anz_eins = 0;
    }
  }
  /// Fügt einem MFX Telegramm Bits hinzu
  /// # Arguments
  /// * bits - Bits, die hinzugefügt werden sollen
  /// * ddl_tel - Telegramm, bei dem Bits hinzugefügt werden sollen
  /// * crc - Aktueller CRC oder CRC Startwert, neuer CRC zurück
  fn add_bits(&mut self, bits: MfxBits, ddl_tel: &mut DdlTel, crc: &mut u8) {
    //Start mit MSB
    let mut maske: u32 = 1 << (bits.1 - 1);
    for _ in 0..bits.1 {
      self.add_bit((bits.0 & maske) != 0, ddl_tel);
      maske >>= 1;
    }
    self.crc(bits, crc);
  }
  /// MFX Sync. zum tel. hinzufügen.
  /// Das Tel. darf nicht leer sein, es muss min ein Byte vorher vorhanden sein.
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem MFX Sync hinzugefügt werden soll
  /// * halb - wenn true wird nur ein halbes Sync.Muster ergänzt (=Pegeländerung)
  fn add_sync(&mut self, ddl_tel: &mut DdlTel, halb: bool) {
    let last = ddl_tel.daten.len() - 1;
    let daten: &mut Vec<u8> = ddl_tel.daten[last].as_mut();
    let values: (u8, u8) = if (*daten.last().unwrap() & 1) == 0 {
      (0x00, 0xFF) //Letzter Pegel war 0 -> "normal"
    } else {
      (0xFF, 0x00) //Letzter Pegel war 1 -> invertiert arbeiten
    };
    daten.push(values.1);
    daten.push(values.1); //0
    daten.push(values.0);
    daten.push(values.1); //1
    daten.push(values.1);
    if !halb {
      daten.push(values.0); //1 mit Regelverlettzung, keine Flanke zu Beginn
      daten.push(values.0);
      daten.push(values.1); //1 mit Regelverlettzung, keine Flanke zu Beginn
      daten.push(values.0);
      daten.push(values.0); //0
    }
    self.anz_eins = 0; //Zähler für Bitstuffimg startet jetzt bei 0
  }
  /// Startpegel / Pause und MFX Sync. zum Tel. hinzufügen.
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem Startpegel / Pause und MFX Sync hinzugefügt werden soll
  fn add_start_sync(&mut self, ddl_tel: &mut DdlTel) {
    let last = ddl_tel.daten.len() - 1;
    for _ in 0..(MFX_STARTSTOP_0_BIT * SPI_BYTES_PRO_BIT) {
      ddl_tel.daten[last].push(0);
    }
    self.add_sync(ddl_tel, false);
  }
  /// CRC zum Tel. hinzufügen.
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem Startpegel / Pause und MFX Sync hinzugefügt werden soll
  /// * crc - aktueller CRC berechnet bis und mit letztes Bit vor CRC.
  fn add_crc(&mut self, ddl_tel: &mut DdlTel, mut crc: u8) {
    //Abschluss der CRC Berechnung ist mit 8 Bits des CRC 0
    self.crc((0, 8), &mut crc);
    self.add_bits((crc as u32, 8), ddl_tel, &mut crc);
  }

  /// CRC und MFX Sync. und Pause zum Abschluss des Tel. hinzufügen.
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem Startpegel / Pause und MFX Sync hinzugefügt werden soll
  /// * crc - aktueller CRC berechnet bis und mit letztes Bit vor CRC.
  fn add_crc_ende_sync(&mut self, ddl_tel: &mut DdlTel, crc: u8) {
    self.add_crc(ddl_tel, crc);
    //Ende Sync
    self.add_sync(ddl_tel, false);
    self.add_sync(ddl_tel, false);
    //Vereinzelte MFX Loks funktionieren nicht zuverlässig. Ausser:
    //- 3. Sync am Ende
    //- Pause nach Abschluss.
    self.add_sync(ddl_tel, false);
    let last = ddl_tel.daten.len() - 1;
    for _ in 0..(MFX_STARTSTOP_0_BIT * SPI_BYTES_PRO_BIT) {
      ddl_tel.daten[last].push(0);
    }
  }
  /// Adresse zum MFX Tel. hinzufügen
  /// Anhänging von der Adresse wird 7, 9, 11, 14 Bit Adressierung verwendet
  /// Liefert den CRC nach Einfügen Adresse zurück.
  /// # Arguments
  /// * adr - Die Adresse
  /// * ddl_tel - Telegramm, bei dem die Schienenadr. hinzugefügt werden soll
  fn add_adr(&mut self, adr: u32, ddl_tel: &mut DdlTel) -> u8 {
    let mut crc: u8 = 0x7F;
    if adr < 128 {
      self.add_bits(MFX_ADR_7_BIT, ddl_tel, &mut crc); //Adr 7 Bit
      self.add_bits((adr, 7), ddl_tel, &mut crc);
    } else if adr < 512 {
      self.add_bits(MFX_ADR_9_BIT, ddl_tel, &mut crc); //Adr 9 Bit
      self.add_bits((adr, 9), ddl_tel, &mut crc);
    } else if adr < 2048 {
      self.add_bits(MFX_ADR_11_BIT, ddl_tel, &mut crc); //Adr 11 Bit
      self.add_bits((adr, 11), ddl_tel, &mut crc);
    } else {
      self.add_bits(MFX_ADR_14_BIT, ddl_tel, &mut crc); //Adr 14 Bit
      self.add_bits((adr, 14), ddl_tel, &mut crc);
    }
    crc
  }
  /// SID - UID zuordnungstelegramm senden
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem die SID Zuordnung hinzugefügt werden soll
  /// * adr - Die Adresse
  fn send_sid(&mut self, ddl_tel: &mut DdlTel, adr: u32) {
    //Format des Bitstreams:
    //10AAAAAAA111011AAAAAAAAAAAAAAUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUCCCCCCCC
    //1.A=0 (Broadcast)
    //2.A die neue Adresse
    //U=32 Bit UID des Dekoder.
    //C=Checksumme
    self.add_start_sync(ddl_tel);
    let mut crc = self.add_adr(0, ddl_tel);
    self.add_bits(MFX_CMD_KONFIG_SID, ddl_tel, &mut crc);
    self.add_bits((adr as u32, 14), ddl_tel, &mut crc);
    self.add_bits((self.uid[adr as usize], 32), ddl_tel, &mut crc);
    self.add_crc_ende_sync(ddl_tel, crc);
  }
  /// MFX Paket mit UID der Zentrale und Neuanmeldezähler versenden.
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem Zentrale UID Telegramm hinzugefügt werden soll
  fn send_uid_regcounter(&mut self, ddl_tel: &mut DdlTel) {
    //Format des Bitstreams:
    //10AAAAAAA111101UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUZZZZZZZZZZZZZZZZCCCCCCCC
    //A=0 (Broadcast)
    //U=32 Bit UID
    //Z=16 Bit Neuanmeldezähler
    //C=Checksumme
    self.add_start_sync(ddl_tel);
    let mut crc = self.add_adr(0, ddl_tel);
    self.add_bits(MFX_CMD_KONFIG_UID, ddl_tel, &mut crc);
    self.add_bits((self.uid_zentrale, 32), ddl_tel, &mut crc);
    self.add_bits((self.reg_counter as u32, 16), ddl_tel, &mut crc);
    self.add_crc_ende_sync(ddl_tel, crc);
  }

  /// MFX Paket zur Suche neuer Dekoder versenden
  /// # Arguments
  /// * ddl_tel: DDL Telegramm bei dem sas Dekodersuchtelegramm hinzugefügt werden soll
  fn send_search_new_decoder(&mut self, ddl_tel: &mut DdlTel) {
    //Format des Bitstreams:
    //10AAAAAAA111010CCCCCCUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUCCCCCCCC
    //A=0 (Broadcast)
    //C=6 Bit Anzahl Bits aus U die im Dekoder übereinstimmen müssen
    //U=32 Bit UID des Dekoder, der gesucht wird.
    //C=Checksumme
    self.add_start_sync(ddl_tel);
    let mut crc = self.add_adr(0, ddl_tel);
    self.add_bits(MFX_CMD_SEARCH_NEW, ddl_tel, &mut crc);
    self.add_bits((self.search_new_dekoder_bits, 6), ddl_tel, &mut crc);
    self.add_bits((self.search_new_dekoder_uid, 32), ddl_tel, &mut crc);
    self.add_crc(ddl_tel, crc);
    //Nun kommt noch der Platz für 1 Bit Rückmeldung
    //11 oder 11.5 Sync
    //Bitfolge "0011"
    //6.4ms Pause mit 0
    //Sync
    //6.4ms Pause mit 1
    //2 Sync
    for _ in [0..11] {
      self.add_sync(ddl_tel, false);
    }
    //Pause geht mit 0 weiter, sollte also hier mit 1 aufhören um letzte Flanke zu haben
    //Wenn nicht: 0.5 Sync ergänzen
    if *ddl_tel.daten.last().unwrap().last().unwrap() == 0 {
      self.add_sync(ddl_tel, true);
    }
    //Bitfolge 0011
    self.add_bits((0b0011, 4), ddl_tel, &mut crc);
    //Pause 6.4ms mit 0
    self.rds_1_bit_start_pos = ddl_tel.daten.last().unwrap().len();
    ddl_tel
      .daten
      .last_mut()
      .unwrap()
      .resize(self.rds_1_bit_start_pos + MFX_LEN_PAUSE_6_4_MS, 0);
    //Sync
    self.add_sync(ddl_tel, false);
    //Pause 6.4ms mit 1
    let len = ddl_tel.daten.last().unwrap().len();
    ddl_tel
      .daten
      .last_mut()
      .unwrap()
      .resize(len + MFX_LEN_PAUSE_6_4_MS, 0xFF);
    //2 Sync
    self.add_sync(ddl_tel, false);
    self.add_sync(ddl_tel, false);
    //Und Rückmeldung aktivieren
    ddl_tel.daten_rx = Some(vec![0; ddl_tel.daten.last().unwrap().len()]);
  }

  /// Auswertung Ergebnis Dekodersuche.
  /// Wenn positives Feedback: weiter mit nächstem Bit, zuerst 0, dann 1.
  /// Wenn 32 Bit gefunden -> neuer Dekoder gefunden.
  /// Ergebnis: siehe "ResultNeuAnmeldung".
  /// # Arguments
  /// * daten_rx: parallel zum Senden eingelesene Daten, Bit = 1 = RDS Feedback war vorhanden
  fn eval_send_search_new_decoder(&mut self, daten_rx: &Vec<u8>) -> ResultNeuAnmeldung {
    let mut result = ResultNeuAnmeldung::None;
    //Rückmeldung wird ers 1ms nach Start Rückmeldefenster ausgewertet da von letzter Schaltflanke
    //noch Schwingungen vorhanden sein könnten die irrtümlich als RDS Signal ausgewertet werden
    const MFX_LEN_PAUSE_1_MS: usize = MFX_LEN_PAUSE_6_4_MS / 6;
    let mut anz_1: u32 = 0;
    for i in self.rds_1_bit_start_pos + MFX_LEN_PAUSE_1_MS
      ..self.rds_1_bit_start_pos + MFX_LEN_PAUSE_6_4_MS - MFX_LEN_PAUSE_1_MS
    {
      anz_1 += u8::count_ones(daten_rx[i]);
    }
    //5% der Bits gesetzt, es wird wohl tatsächlich ein Dekoder geantwortet haben und keine Störung sein
    //War 50% bis PIKO Giruno, hier kommt ein viel schwächeres Signal ... :-(
    if anz_1 >= (((MFX_LEN_PAUSE_6_4_MS - MFX_LEN_PAUSE_1_MS) * 8 / 20) as u32) {
      //Positive Rückmeldung erhalten
      result = ResultNeuAnmeldung::InProgress;
      //Wenn bereits 32 Bit gefunden -> Neuer Dekoder gefunden
      if self.search_new_dekoder_bits >= 32 {
        if self.search_new_dekoder_uid == 0 {
          warn!("MFX Dekodersuche Fehler. UID 0 wird ignoriert.");
          result =
            ResultNeuAnmeldung::Error("MFX Dekodersuche Fehler. UID 0 wird ignoriert.".to_string());
        } else {
          info!(
            "MFX Dekodersuche neu gefunden UID {}",
            self.search_new_dekoder_uid
          );
          result = ResultNeuAnmeldung::Ok(self.search_new_dekoder_uid);
          //Und Neuanmeldezähler inkrementieren
          self.reg_counter += 1;
          self.save_registration_counter();
        }
        //Für neue Suche bereit machen
        self.search_new_dekoder_uid = 0;
        self.search_new_dekoder_bits = 0;
      } else {
        info!(
          "MFX Dekodersuche UID Bit gefunden. Anzahl Bits gefunden {} UID {}",
          self.search_new_dekoder_bits, self.search_new_dekoder_uid
        );
        // mit nächstem Bit weiter suchen
        self.search_new_dekoder_bits += 1;
      }
    } else {
      //Wenn die Suche einen Dekoder gefunden hat und das aktuelle Bit 0 war, dann kann nun noch mit 1 probiert werden
      if self.search_new_dekoder_bits > 0 {
        let bit = 0x80000000 >> (self.search_new_dekoder_bits - 1);
        if (self.search_new_dekoder_uid & bit) == 0 {
          self.search_new_dekoder_uid |= bit;
        } else {
          //Weder 0 noch 1 haben zu einer positiven Antwort geführt -> Abbruch
          warn!(
            "Abbruch MFX Dekodersuche. Keine Antwort mehr bei Bit {} Aktuelle UID {}",
            self.search_new_dekoder_bits, self.search_new_dekoder_uid
          );
          self.search_new_dekoder_uid = 0;
          self.search_new_dekoder_bits = 0;
        }
      }
    }
    result
  }

  /// Liefert ein MFX CV Read/Write Telegramm.
  /// # Arguments
  /// * tel - Zu erzeugendes Telegramm
  fn get_cv_tel(&mut self, tel: &MfxCvTel) -> DdlTel {
    //Format des Bitstreams:
    //Read:  10AAAAAAA111000VVVVVVVVVVIIIIIIBBCCCCCCCC
    //Write: 10AAAAAAA111001VVVVVVVVVVIIIIIIBBDDDDDDDDCCCCCCCC
    //A=Adresse
    //V=10 Bit CV Adresse
    //I=6 Bit Index
    //B=2 Bit Anzahl Bytes 00=1, 01=2, 10=4, 11=8 (Beim Schreiben unterstützen Dekoder aber nur ein Byte, hier ist jedoch alles implementiert)
    //D=8(bis 64) Bit Daten zum Schreiben
    //C=Checksumme
    let mut ddl_tel = self.get_gl_new_tel(tel.adr, true, tel.trigger); //Refresh->nur einmaliges Senden
    self.add_start_sync(&mut ddl_tel);
    let mut crc = self.add_adr(tel.adr, &mut ddl_tel);
    self.add_bits(
      match tel.mfx_cv_type {
        MfxCvTelType::Read => MFX_CMD_CV_READ,
        MfxCvTelType::Write(_) => MFX_CMD_CV_WRITE,
      },
      &mut ddl_tel,
      &mut crc,
    );
    self.add_bits((tel.cv as u32, 10), &mut ddl_tel, &mut crc);
    self.add_bits((tel.index as u32, 6), &mut ddl_tel, &mut crc);
    self.add_bits((tel.byte_count.mfx_code(), 2), &mut ddl_tel, &mut crc);
    let byte_count = tel.byte_count.byte_count();
    if let MfxCvTelType::Write(value) = &tel.mfx_cv_type {
      for i in 0..byte_count {
        self.add_bits((value[i] as u32, 8), &mut ddl_tel, &mut crc);
      }
    }
    self.add_crc(&mut ddl_tel, crc);
    if tel.mfx_cv_type == MfxCvTelType::Read {
      //Nun kommt noch der Platz für RDS Rückmeldung
      //11 oder 11.5 Sync
      //Bitfolge "0011"
      //23 Takte 25us alle 912us
      //(3+AnzBits+8+4)*2 Takte 25us alle 456us (3 Bit Startkennung, Daten, 8 Bit Checksumme, 4 zusätzlich am Ende)
      //2 Sync (eigentlich reicht einer, mit 2 ist sicher beim ersten auch die abschliessende Flanke vorhanden)
      for _ in 0..11 {
        self.add_sync(&mut ddl_tel, false);
      }
      //Pause geht mit 0 weiter, sollte also hier mit 1 aufhören um letzte Flanke zu haben
      //Wenn nicht: 0.5 Sync ergänzen
      if *ddl_tel.daten.last().unwrap().last().unwrap() == 0 {
        self.add_sync(&mut ddl_tel, true);
      }
      //Bitfolge 0011
      self.add_bits((0b0011, 4), &mut ddl_tel, &mut crc);
      //23 Takte 25us alle 912us
      //Ab hier wird mit einzelnen Bits gearbeitet um das verlangte Timing möglichst genau einzuhalten
      //MSB wird zuerst ausgegeben
      let daten = ddl_tel.daten.last_mut().unwrap();
      //Anzahl Bits die im letzten Bytes noch frei sind
      let mut anz_bit_frei: usize = 0;
      for _ in 0..23 {
        //Pause mit Ruhepegel
        self.add_single_bits(
          daten,
          &mut anz_bit_frei,
          BITS_PER_RDSSYNC_PAUSE_PERIOD,
          false,
        );
        //Pause ist da, nun der Impuls
        self.add_single_bits(daten, &mut anz_bit_frei, BITS_PER_RDSSYNC_IMP, true);
      }
      //(3+AnzBits+8+4)*2 Takte 25us alle 456us
      for _ in 0..(3 + byte_count * 8 + 8 + 4) * 2 {
        //Pause mit Ruhepegel
        self.add_single_bits(daten, &mut anz_bit_frei, BITS_PER_RDSSYNC_PAUSE_HALF, false);
        //Pause ist da, nun der Impuls
        self.add_single_bits(daten, &mut anz_bit_frei, BITS_PER_RDSSYNC_IMP, true);
      }
      //Und noch eine letzte Pause
      //Pause mit Ruhepegel
      self.add_single_bits(daten, &mut anz_bit_frei, BITS_PER_RDSSYNC_PAUSE_HALF, false);
    }
    //2 Sync
    self.add_sync(&mut ddl_tel, false);
    self.add_sync(&mut ddl_tel, false);
    ddl_tel
  }

  /// Einzelne Bits zu einem Vector hinzufügen
  /// # Arguments
  /// * daten - Vector, bei dem Bits hinzugefügt werden sollen.
  /// * anzBitFrei - Wieviel Bits sind im letzten Byte noch frei als Inputs und dann als Output
  ///                Start Daten beim MSB, das heisst freie Bits beim LSB.
  /// * anzBits - Anzahl der Bits, die hinzugefügt werden sollen.
  /// * bitValue - true: 1, false: 0 hinzufügen
  fn add_single_bits(
    &self, daten: &mut Vec<u8>, anz_bit_frei: &mut usize, anz_bits: usize, bit_value: bool,
  ) {
    for _ in 0..anz_bits {
      //Wenn im letzten Byte noch etwas frei ist -> verwenden
      if *anz_bit_frei > 0 {
        //Ein Bit Weniger Frei
        *anz_bit_frei -= 1;
      } else {
        //Jetzt braucht es ein neues Byte
        daten.push(0);
        //Noch 7 Bits frei
        *anz_bit_frei = 7;
      }
      //Nun noch verlangten Wert auf 1 setzen wenn 1 verlangt. Bei 0 nicht machen, Byte wurde als 0 hinzugefügt
      if bit_value {
        let letztes_byte = daten.last_mut().unwrap();
        *letztes_byte |= 1 << *anz_bit_frei;
      }
    }
  }
}

impl DdlProtokoll for MfxProtokoll {
  /// Legt fest, ob das Protokoll eine UID benötigt, die bei GL INIT Kommando angegeben werden muss
  /// Return true wenn UID benötigt.
  fn uid(&self) -> bool {
    true
  }
  /// GL Init Daten setzen. Welche Daten verwendet werden ist Protokollabhängig.
  /// Liefert, wenn "power" ein allfällg notwendiges Init-Telegramm (z.B. MFX SID Zurordnung) zurück.
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * uid - UID des Dekoders, wenn vorhanden
  /// * funk_anz - Anzahl tatsächlich verwendete Funktionen. Kann, je nach Protokoll, dazu
  ///              verwendet werden, nur Telegramme der verwendeten Funktionen zu senden.
  /// * power - true wenn Power ein, ein allfällig notwendiges Telegramm (z.B. MFX Schienenadr. Zuordung)
  ///           wird zum sofort senden zurückgegeben.
  ///           false wenn Power aus, nie direkte Ausgabe notwendiges Init Telegramm, dieses muss vor nächstem
  ///           GL Befehl ausgegeben werden.
  /// * trigger - Oszi Trigger?
  fn init_gl(
    &mut self, adr: u32, uid: Option<u32>, funk_anz: usize, power: bool, trigger: bool,
  ) -> Option<DdlTel> {
    self.uid[adr as usize] = uid.unwrap();
    self.funk_anz[adr as usize] = funk_anz;
    //Merken, dass vor nächstem Lokbefehl noch neue Schienenadr. Zuordnung gesendet werden muss.
    //Wird nicht hier direkt gemacht, da Init auch bei Booster Stop ausgeführt wird.
    self.new_sid[adr as usize] = true;
    if power {
      //SID Zuordnungstelegramm kann gleich ausgegeben werden
      let mut ddl_tel = self.get_gl_new_tel(adr, false, trigger);
      //Nur SID Telegramm ist relevant, kein weiteres notwendig
      ddl_tel.daten.truncate(1);
      Some(ddl_tel)
    } else {
      None
    }
  }
  /// Liefert die max. erlaubte Lokadresse
  fn get_gl_max_adr(&self) -> u32 {
    MAX_MFX_GL_ADRESSE
  }
  /// Wieviele Speedsteps werden vom Protokoll unterstützt
  /// Maximale Möglichkeit MFX: 127
  fn get_gl_max_speed_steps(&self) -> usize {
    127
  }

  /// Liefert die max. erlaubte Schaltmoduladdresse
  /// MFX unterstützt keine GA, deshalb 0.
  fn get_ga_max_adr(&self) -> u32 {
    0
  }

  /// Liefert die max. Anzahl der unterstützten Funktionen
  fn get_gl_anz_f(&self) -> usize {
    64 //Eigentlich kann MFX Total 128 (F0-F127), im Moment reicht mir das aber, die Funktionen werden in ganz srcp_rust in einem u64 verwaltet
  }

  /// Liefert die Anzahl Funktionen (inkl. F0) die im Basistelegramm enthalten sind
  /// Muss immer <= "get_Anz_F" sein.
  fn get_gl_anz_f_basis(&self) -> usize {
    16 //Alle, die direkt geschaltet werden können
  }

  /// Liefert ein neues GL Telegramm zur Verwendung in "get_gl_basis_tel" und / oder "get_gl_zusatz_tel".
  /// Falls für GL "adr" noch ein neues SID Zuordnungstelegramm gesendet werden muss, ist dieses bereits
  /// enthalten, ansonsten ist noch nichts enthalten.
  /// # Arguments
  /// * adr - Adresse der Lok zur Erkennung, ob noch SID Zuordung gesendet werden muss
  /// * refresh - Wenn true: Aufruf aus Refres Cycle, einmalige Telegramm Versendung,
  ///             Wenn false: Aufruf wegen neuem Lokkommando, mehrmaliges Versenden
  /// * trigger - Oszi Trigger?
  fn get_gl_new_tel(&mut self, adr: u32, refresh: bool, trigger: bool) -> DdlTel {
    let mut ddl_tel = DdlTel::new(
      adr,
      SPI_BAUDRATE_MFX_2,
      Duration::ZERO,
      false,
      MFX_MAX_LEN,
      if refresh { 1 } else { 2 }, //Neue Telegramme 2-fach senden
      trigger,
    );
    if self.new_sid[adr as usize] {
      self.send_sid(&mut ddl_tel, adr);
      ddl_tel.daten.push(Vec::with_capacity(MFX_MAX_LEN));
      self.new_sid[adr as usize] = false;
    }
    ddl_tel
  }

  /// Erzeugt das Basis Telegramm für GL.
  /// - Fahren
  /// - Basisfunktionen F0 bis "get_Anz_F_Basis". Es wedren hier nur diese Funktionen übernommen!
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * drive_mode - Fahrtrichtung / Nothalt
  /// * speed - aktuelle Geschwindigkeit
  /// * speed_steps - Anzahl Speed Steps die verwendet werden soll. Protokoll abhängig.
  ///                 Wird hier nicht verwendet, bei MFX wird immer mit 127 Stufen gefahren
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt bis "get_Anz_F_Basis"
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_gl_basis_tel(
    &mut self, adr: u32, drive_mode: GLDriveMode, speed: usize, _speed_steps: usize,
    funktionen: u64, ddl_tel: &mut DdlTel,
  ) {
    self.add_start_sync(ddl_tel);
    //Speed 1 = Nothalt
    let speed_used = if drive_mode == GLDriveMode::Nothalt {
      1
    } else {
      if speed == 1 {
        2
      } else {
        speed
      }
    };
    //Drivemode für Richtung, wenn Nothalt dann der letzte
    let drive_mode_used: GLDriveMode = if drive_mode == GLDriveMode::Nothalt {
      self.old_drive_mode[adr as usize]
    } else {
      drive_mode
    };
    self.old_drive_mode[adr as usize] = drive_mode_used;
    //Format des Bitstreams für 127 Fahrstufen ist:
    //<=4 Fn       : ..A001RSSSSSSS010FFFFCCCCCCCC
    //>4 .. <=8 Fn : ..A001RSSSSSSS0110FFFFFFFFCCCCCCCC
    //>8 Fn        : ..A001RSSSSSSS0111FFFFFFFFFFFFFFFFCCCCCCCC
    //Bei speed==0 : ..A000RSSS.....
    //A=Adresse, 7, 9, 11 oder 14 Bit
    //R=Richtung
    //S=Fahrstufe
    //F=F0 bis F15
    //C=Checksumme
    let mut crc = self.add_adr(adr as u32, ddl_tel);
    if speed_used == 0 {
      //Kurzes Fahren Kommando verwenden
      self.add_bits(MFX_CMD_FAHREN_KURZ, ddl_tel, &mut crc);
      self.add_bits(
        (
          if drive_mode_used == GLDriveMode::Vorwaerts {
            0
          } else {
            1
          },
          1,
        ),
        ddl_tel,
        &mut crc,
      );
      self.add_bits((0, 3), ddl_tel, &mut crc);
    } else {
      //Langes Fahren Kommando 7 Bit verwenden
      self.add_bits(MFX_CMD_FAHREN, ddl_tel, &mut crc);
      self.add_bits(
        (
          if drive_mode_used == GLDriveMode::Vorwaerts {
            0
          } else {
            1
          },
          1,
        ),
        ddl_tel,
        &mut crc,
      );
      self.add_bits((speed_used as u32, 7), ddl_tel, &mut crc);
    }
    if self.funk_anz[adr as usize] <= 4 {
      self.add_bits(MFX_CMD_FNKT_F0_F3, ddl_tel, &mut crc);
      self.add_bits((funktionen as u32, 4), ddl_tel, &mut crc);
    } else if self.funk_anz[adr as usize] <= 8 {
      self.add_bits(MFX_CMD_FNKT_F0_F7, ddl_tel, &mut crc);
      self.add_bits((funktionen as u32, 8), ddl_tel, &mut crc);
    } else {
      self.add_bits(MFX_CMD_FNKT_F0_F15, ddl_tel, &mut crc);
      self.add_bits((funktionen as u32, 16), ddl_tel, &mut crc);
    }
    self.add_crc_ende_sync(ddl_tel, crc);
    //F0 bis 15 übernehmen
    self.old_funktionen[adr as usize] &= !0xFFFF;
    self.old_funktionen[adr as usize] |= funktionen & 0xFFFF;
  }

  /// Erzeugt das / die Fx Zusatztelegramm(e) für GL.
  /// - Funktionen nach "get_Anz_F_Basis"
  /// Alle notwendigen Telegramme werden einzeln zu "ddl_tel" hinzugefügt.
  /// Wenn keine extra Telegramme für Zusatzfunktionen vorhanden sind, dann wird nichts hinzugefügt.
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * refresh - Wenn false werden nur Telegramme für Funktionen, die geändert haben, erzeugt
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt ab "get_Anz_F_Basis"
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_gl_zusatz_tel(&mut self, adr: u32, refresh: bool, funktionen: u64, ddl_tel: &mut DdlTel) {
    if self.funk_anz[adr as usize] <= self.get_gl_anz_f_basis() {
      //Hier gibt es nichts zu tun
      return;
    }
    //Format des Bitstreams ist:
    //..A100NNNNNNN0FCCCCCCCC
    //A=Adresse, 7, 9, 11 oder 14 Bit
    //N=F0..127
    //F=Funktion Ein/Aus 0/1
    //C=Checksumme
    for i in self.get_gl_anz_f_basis()..self.funk_anz[adr as usize] {
      if refresh || (((self.old_funktionen[adr as usize] ^ funktionen) & (1 << i)) != 0) {
        //Refresh mit zwingend alle senden oder Veränderung
        self.add_start_sync(ddl_tel);
        let mut crc = self.add_adr(adr as u32, ddl_tel);
        self.add_bits(MFX_CMD_FNKT_EINZELN, ddl_tel, &mut crc);
        self.add_bits((i as u32, 7), ddl_tel, &mut crc);
        self.add_bits(
          (
            if funktionen & (1 << i) == 0 {
              0b00
            } else {
              0b01
            },
            2,
          ),
          ddl_tel,
          &mut crc,
        );
        self.add_crc_ende_sync(ddl_tel, crc);
        ddl_tel.daten.push(Vec::with_capacity(MFX_MAX_LEN));
      }
    }
    //Falls nun am Schluss noch ein leeres vorhanden ist kann dieses entfernt werden
    if ddl_tel.daten.last().unwrap().is_empty() {
      ddl_tel.daten.resize(ddl_tel.daten.len() - 1, vec![]);
    }
  }

  /// Liefert ein leeres GA Telegramm zur Verwendung in "get_ga_tel".
  /// Nicht verwendet, keine GA's in MFX
  /// # Arguments
  /// * adr - Adresse GA, keine Verwendunbg, nur Debug Support
  /// * trigger - Oszi Trigger
  fn get_ga_new_tel(&self, adr: u32, trigger: bool) -> DdlTel {
    assert!(false, "MFX unterstützt keine GA, Aufruf get_ga_new_tel");
    DdlTel::new(
      adr,
      SPI_BAUDRATE_MFX_2,
      Duration::ZERO,
      false,
      0,
      1,
      trigger,
    )
  }

  /// Erzeugt ein GA Telegramm
  /// Liefert true zurück, wenn Timeout zu r automatischen Abschaltung durch Protokoll / Dekoder übernommen wird.
  /// # Arguments
  /// * adr - Adresse des Schaltdekoders
  /// * port - Port auf dem Schaltdekoder
  /// * value - Gewünschter Zustand des Port Ein/Aus
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  /// * value - Gewünschter Zustand des Port Ein/Aus (0/1) oder Begriff (z.B. Erweiterte DCC Dekoder)
  /// * timeout - Wenn das Protokoll eine automatische Ausschaltung des Ausgangs durch den Dekoder unterstützt kann hier die Zeit in ms angegeben werden.
  ///             None = kein Timeout, dauerhaft schalten. 
  ///             Duration::ZERO = Port ignorieren, Value ist der zu sendende Begriff (z.B. Erweiterte Funktionsdekoder NMRA/DCC Signalbegriff)
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  /// Nicht verwendet, keine GA's in MFX
  fn get_ga_tel(&self, _adr: u32, _port: usize, _value: usize, _timeout: Option<Duration>, _ddl_tel: &mut DdlTel) -> bool {
    assert!(false, "MFX unterstützt keine GA, Aufruf get_ga_tel");
    false
  }

  /// Liefert das Idle Telegramm dieses Protokolles
  /// Return None wenn kein Idle Telegramm vorhanden ist
  /// MFX hat kein Idle Telegramm. Als Idle wird UID der Zentrale und Dekoder Abfrage gesendet
  /// Sobald eine GL vorhanden ist, wird keine Idle mehr gesendet, UID Zentrale wird dann periodisch
  /// über get_protokoll_telegrammme im Intervall INTERVALL_UID gesendet
  fn get_idle_tel(&mut self) -> Option<DdlTel> {
    let mut ddl_tel = self.get_gl_new_tel(0, true, false); //Refresh->nur einmaliges Senden
    self.send_uid_regcounter(&mut ddl_tel);
    Some(ddl_tel)
  }
  /// Liefert zusätzliche, Protokoll spezifische Telegramme (z.B. bei MFX die UID & Neuanmeldezähler der Zentrale)
  /// Liefert None, wenn es nichts zur versenden gibt
  /// Hier für MFX wird periodisch die UID / Neuanmeldezähler der Zentrale und Suche
  /// nach noch nicht angemeldeten Dekodern versandt.
  /// Zudem werden die CV Read/Write Telegramme erzeugt, wenn vom MFX RDS Thread verlangt.
  /// Vorrang hat CV Read/Write. Wenn das verlangt wird, dann wird nie Zentrale UID und Dekodersuche erzeugt.
  /// Das kann ein Zyklus später erfolgen falls gleichzeitig notwendig.
  /// # Arguments
  /// * power : true wenn Power (Booster) ein, sonst false.
  ///           Normalerweise werden Telegramme nur bei Power On gesendet.
  ///           Ausnahme: SM DCC auf Prog. Gleis.
  ///           Hier: kein Prog. Gleis für MFX, alle Telegramme sind nur sinnvoll wenn Power On ist.
  fn get_protokoll_telegrammme(&mut self, power: bool) -> Option<DdlTel> {
    if !power {
      //Bei Power Off gibt es hier nichts zu senden
      return None;
    }
    let mut result = None;
    let tel_from_rds = self.rx_tel_from_rds.try_recv();
    if let Ok(tel) = tel_from_rds {
      result = Some(self.get_cv_tel(&tel));
    } else {
      let now = Instant::now();
      if now >= (self.zeitpunkt_uid + INTERVALL_UID) {
        self.zeitpunkt_uid = now;
        //UID Zentrale und Neuanmeldezähler
        let mut tel = self.get_idle_tel().unwrap();
        //Keine Suche wenn SM aktiv ist oder bereits eine Lokanmeldung läuft
        if !self.sm_aktiv && self.read_gl_parameter.is_none() {
          //Suche neue Dekoder, nächstes Telegramm
          tel.daten.push(Vec::with_capacity(MFX_MAX_LEN_SEARCH_NEW));
          self.send_search_new_decoder(&mut tel);
        }
        result = Some(tel);
      }
    }
    result
  }

  /// Auswertung automatische Dekoderanmeldung (z.B. bei MFX).
  /// Notwendige Telegramme zur Suche müssen über "get_protokoll_telegrammme" ausgegeben und eine Rückmeldung
  /// verlangt werden.
  /// Wenn ein neuer Dekoder gefunden wurde, dann wird dessen UID zurückgegeben, ansonsten der aktuelle Zustand, siehe "ResultNeuAnmeldung".
  /// # Arguments
  /// * daten_rx : Die beim parallel zum Senden über SPI eingelesenen Daten
  fn eval_neu_anmeldung(&mut self, daten_rx: &Vec<u8>) -> ResultNeuAnmeldung {
    self.eval_send_search_new_decoder(daten_rx)
  }

  /// Auslesen optionale GL Parameter (z.B. MFX Lokname und Funktionen)
  /// Liefert ResultReadGlParameter::Error zurück, wenn ein Fehler aufgetreten ist oder vom Protokoll nicht untertsützt.
  /// Liefert ResultReadGlParameter::Busy zurück, wenn das Auslesen im Gange ist
  /// Liefert ResultReadGlParameter::Ok mit den Parametern zurück, wenn abgeschlossen.
  /// Falls "eval_neu_anmeldung" "Some" liefert, sollte hier nicht Error zurückgegeben werden.
  /// Falls vom Protokoll nicht unterstützt, dann Ok mit leerer Liste.
  /// # Arguments
  /// * adr : Schienenadresse der GL
  fn read_gl_parameter(&mut self, adr: u32) -> ResultReadGlParameter {
    let mut result = ResultReadGlParameter::Busy;
    if let Some(adr_read_gl_parameter) = self.read_gl_parameter {
      if adr_read_gl_parameter == adr {
        //Ist ein Ergebnis vorhanden?
        if let Ok(init_parameter) = self.rx_from_rds_lok_init.try_recv() {
          //Antwort vorhanden
          if init_parameter.is_some() {
            //Auslesen hat funktioniert
            info!("MFX Start read GL Parameter fertig Adr={}", adr);
            result = ResultReadGlParameter::Ok(init_parameter.unwrap());
            //Fertig
            self.read_gl_parameter = None;
          } else {
            //Fehler, Abbruch
            warn!("MFX Error read GL Parameter Adr={}", adr);
            result = ResultReadGlParameter::Error;
            self.read_gl_parameter = None;
          }
        }
      } else {
        //Auslesen im Gange, es kann nicht gleichzeitig eine andere Adresse ausgelesen werden
        warn!(
          "MFX Error read GL Parameter Adr={} aber bereits Adr={} in arbeit",
          adr, adr_read_gl_parameter
        );
        result = ResultReadGlParameter::Error;
      }
    } else {
      //Auslesen Lokparameter starten
      info!("MFX Start read GL Parameter Adr={}", adr);
      self.read_gl_parameter = Some(adr);
      self
        .tx_to_rds
        .send(MfxRdsJob::new_read_all_init_parameter(adr))
        .unwrap();
    }
    result
  }

  /// Dekoderkonfiguration (SM) Start
  /// # Arguments
  /// * smParameter : Optinal weiterer Protokollspezifischer Parameter -> hier nicht verwendet
  fn sm_init(&mut self, _sm_parameter: Option<&str>) {
    self.sm_aktiv = true;
  }
  /// Dekoderkonfiguration (SM) Ende
  fn sm_term(&mut self) {
    self.sm_aktiv = false;
  }

  /// Dekoderkonfiguration (SM) Read/Write Value.
  /// # Arguments
  /// * sm_para - Alle notwndigen Paramater für SM Read/Write
  fn sm_read_write(&mut self, sm_para: &SmReadWrite) {
    self
      .tx_to_rds
      .send(MfxRdsJob::ReadWriteCA {
        ca_parameter: sm_para.clone(),
      })
      .unwrap();
  }
  /// Liefert alle in "sm_read" und "sm_write" unterstützten Typen mit der Anzahl erwarteter Parameter
  /// ohne Value für SET.
  /// Wenn diese Funktion Some zurück gibt, dann müssen sowohl "sm_read" als auch "sm_write" bei
  /// entsprechendem Aufruf "Some" zurückgeben!
  /// None wenn SM nicht unterstützt wird.
  fn sm_get_all_types(&self) -> Option<HashMap<String, usize>> {
    let mut result: HashMap<String, usize> = HashMap::new();
    //4 Parameter bei Zugriff auf MFX Konfigvariabeln: Block, CA, CA_Index, Index
    result.insert("CAMFX".to_string(), 4);
    Some(result)
  }
  /// Liefert die Antwort sm_read_write zurück.
  /// None wenn keine Antwort verfügbar.
  fn sm_get_answer(&mut self) -> Option<SmReadWrite> {
    self.rx_from_rds_read_write_ca.try_recv().ok()
  }
}
