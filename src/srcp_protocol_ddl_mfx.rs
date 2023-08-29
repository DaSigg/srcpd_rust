use std::{
  fs,
  time::{Duration, Instant},
};

use log::{info, warn};

use crate::srcp_protocol_ddl::{DdlProtokoll, DdlTel, GLDriveMode};

//SPI Baudrate für MFX.
//Auch hier müssen wir auf sicher 96 Bytes kommen um im DMA Modus zu sein und keine Pause zwischen den Bytes zu haben.
//1 Bit in MFX sind immer 100us. 1 Bit wird auf ein SPI Byte gelegt, also für ein Bit 100 / 8 = 12.5us -> 80000 Baud
//Grösse Paket wird damit (mit Sync Muster) für das kleinst mögliche Paket, das wir senden:
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
const MAX_MFX_GL_ADRESSE: usize = 2_usize.pow(14) - 1;

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
const MFX_MAX_LEN: usize = (MFX_STARTSTOP_0_BIT + 5 + 54 + 8 + 15 + MFX_STARTSTOP_0_BIT) * 2;

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
  old_drive_mode: [GLDriveMode; MAX_MFX_GL_ADRESSE + 1],
  /// Erkennung Funktionswechsel für die nicht immer gesendeten höheren Fx
  old_funktionen: [u64; MAX_MFX_GL_ADRESSE + 1],
  /// Dekoder UID's
  uid: [u32; MAX_MFX_GL_ADRESSE + 1],
  /// Anzahl Initialisierte Funktionen
  funk_anz: [usize; MAX_MFX_GL_ADRESSE + 1],
  /// Muss neue Schienenadr. Zuordung gesendet werden?
  new_sid: [bool; MAX_MFX_GL_ADRESSE + 1],
  /// Anzahl aufeinander folgende 1er für MFX Bitstuffing. Wird mit "add_start_sync" auf 0 gesetzt.
  anz_eins: usize,
  /// Zeitpunkt letztes Versenden UID Zentrale
  zeitpunkt_uid: Instant,
}

impl MfxProtokoll {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * version - V1 oder V2
  /// * uid_zentrale - Zu verwendende UID der Zentrale
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
    MfxProtokoll {
      _version: version,
      uid_zentrale,
      reg_counter,
      path_reg_counter_file,
      old_drive_mode: [GLDriveMode::Vorwaerts; MAX_MFX_GL_ADRESSE + 1],
      old_funktionen: [0; MAX_MFX_GL_ADRESSE + 1],
      uid: [0; MAX_MFX_GL_ADRESSE + 1],
      funk_anz: [0; MAX_MFX_GL_ADRESSE + 1],
      new_sid: [false; MAX_MFX_GL_ADRESSE + 1],
      anz_eins: 0,
      zeitpunkt_uid: Instant::now(),
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
    let values: (u8, u8) = if *daten.last().unwrap() == 0 {
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
  fn add_sync(&mut self, ddl_tel: &mut DdlTel) {
    let last = ddl_tel.daten.len() - 1;
    let daten: &mut Vec<u8> = ddl_tel.daten[last].as_mut();
    let values: (u8, u8) = if *daten.last().unwrap() == 0 {
      (0x00, 0xFF) //Letzter Pegel war 0 -> "normal"
    } else {
      (0xFF, 0x00) //Letzter Pegel war 1 -> invertiert arbeiten
    };
    daten.push(values.1);
    daten.push(values.1); //0
    daten.push(values.0);
    daten.push(values.1); //1
    daten.push(values.1);
    daten.push(values.0); //1 mit Regelverlettzung, keine Flanke zu Beginn
    daten.push(values.0);
    daten.push(values.1); //1 mit Regelverlettzung, keine Flanke zu Beginn
    daten.push(values.0);
    daten.push(values.0); //0
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
    self.add_sync(ddl_tel);
  }
  /// CRC und MFX Sync. und Pause zum Abschluss des Tel. hinzufügen.
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem Startpegel / Pause und MFX Sync hinzugefügt werden soll
  /// * crc - aktueller CRC berechnet bis und mit letztes Bit vor CRC.
  fn add_crc_ende_sync(&mut self, ddl_tel: &mut DdlTel, mut crc: u8) {
    //Abschluss der CRC Berechnung ist mit 8 Bits des CRC 0
    self.crc((0, 8), &mut crc);
    self.add_bits((crc as u32, 8), ddl_tel, &mut crc);
    //Ende Sync
    self.add_sync(ddl_tel);
    self.add_sync(ddl_tel);
    //Vereinzelte MFX Loks funktionieren nicht zuverlässig. Ausser:
    //- 3. Sync am Ende
    //- Pause nach Abschluss.
    self.add_sync(ddl_tel);
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
  fn send_sid(&mut self, ddl_tel: &mut DdlTel, adr: usize) {
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
    self.add_bits((self.uid[adr], 32), ddl_tel, &mut crc);
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
}

impl DdlProtokoll for MfxProtokoll {
  /// Legt fest, ob das Protokoll eine UID benötigt, die bei GL INIT Kommando angegeben werden muss
  /// Return true wenn UID benötigt.
  fn uid(&self) -> bool {
    true
  }
  /// GL Init Daten setzen. Welche Daten verwendet werden ist Protokollabhängig.
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * uid - UID des Dekoders
  /// * funk_anz - Anzahl tatsächlich verwendete Funktionen. Kann, je nach Protokoll, dazu
  ///              verwendet werden, nur Telegramme der verwendeten Funktionen zu senden.
  fn init_gl(&mut self, adr: usize, uid: u32, funk_anz: usize) {
    self.uid[adr] = uid;
    self.funk_anz[adr] = funk_anz;
    //Merken, dass vor nächstem Lokbefehl noch neue Schienenadr. Zuordnung gesendet werden muss.
    //Wird nicht hier direkt gemacht, da Init auch bei Booster Stop ausgeführt wird.
    self.new_sid[adr] = true;
  }
  /// Liefert die max. erlaubte Lokadresse
  fn get_gl_max_adr(&self) -> usize {
    MAX_MFX_GL_ADRESSE
  }

  /// Liefert die max. erlaubte Schaltmoduladdresse
  /// MFX unterstützt keine GA, deshalb 0.
  fn get_ga_max_adr(&self) -> usize {
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

  /// Liefert ein leeres GL Telegramm zur Verwendung in "get_gl_basis_tel" und / oder "get_gl_zusatz_tel".
  /// # Arguments
  /// * adr - Adresse der Lok, keine Verwendunbg, nur Debug Support
  /// * refresh - Wenn true: Aufruf aus Refres Cycle, einmalige Telegramm Versendung,
  ///             Wenn false: Aufruf wegen neuem Lokkommando, mehrmaliges Versenden
  fn get_gl_new_tel(&self, adr: usize, refresh: bool) -> DdlTel {
    DdlTel::new(
      adr,
      SPI_BAUDRATE_MFX_2,
      Duration::ZERO,
      false,
      MFX_MAX_LEN,
      if refresh { 2 } else { 1 }, //Neue Telegramme 2-fach senden
    )
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
    &mut self, adr: usize, drive_mode: GLDriveMode, speed: usize, _speed_steps: usize,
    funktionen: u64, ddl_tel: &mut DdlTel,
  ) {
    if self.new_sid[adr] {
      self.send_sid(ddl_tel, adr);
      ddl_tel.daten.push(Vec::with_capacity(MFX_MAX_LEN));
      self.new_sid[adr] = false;
    }
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
      self.old_drive_mode[adr]
    } else {
      drive_mode
    };
    self.old_drive_mode[adr] = drive_mode_used;
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
    if self.funk_anz[adr] <= 4 {
      self.add_bits(MFX_CMD_FNKT_F0_F3, ddl_tel, &mut crc);
      self.add_bits((funktionen as u32, 4), ddl_tel, &mut crc);
    } else if self.funk_anz[adr] <= 8 {
      self.add_bits(MFX_CMD_FNKT_F0_F7, ddl_tel, &mut crc);
      self.add_bits((funktionen as u32, 8), ddl_tel, &mut crc);
    } else {
      self.add_bits(MFX_CMD_FNKT_F0_F15, ddl_tel, &mut crc);
      self.add_bits((funktionen as u32, 16), ddl_tel, &mut crc);
    }
    self.add_crc_ende_sync(ddl_tel, crc);
    //F0 bis 15 übernehmen
    self.old_funktionen[adr] &= !0xFFFF;
    self.old_funktionen[adr] |= funktionen & 0xFFFF;
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
  fn get_gl_zusatz_tel(
    &mut self, adr: usize, refresh: bool, funktionen: u64, ddl_tel: &mut DdlTel,
  ) {
    if self.funk_anz[adr] <= self.get_gl_anz_f_basis() {
      //Hier gibt es nichts zu tun
      return;
    }
    //Format des Bitstreams ist:
    //..A100NNNNNNN0FCCCCCCCC
    //A=Adresse, 7, 9, 11 oder 14 Bit
    //N=F0..127
    //F=Funktion Ein/Aus 0/1
    //C=Checksumme
    for i in self.get_gl_anz_f_basis()..self.funk_anz[adr] {
      if refresh || (((self.old_funktionen[adr] ^ funktionen) & (1 << i)) != 0) {
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
  fn get_ga_new_tel(&self, adr: usize) -> DdlTel {
    assert!(false, "MFX unterstützt keine GA, Aufruf get_ga_new_tel");
    DdlTel::new(adr, SPI_BAUDRATE_MFX_2, Duration::ZERO, false, 0, 1)
  }

  /// Erzeugt ein GA Telegramm
  /// # Arguments
  /// * adr - Adresse des Schaltdekoders
  /// * port - Port auf dem Schaltdekoder
  /// * value - Gewünschter Zustand des Port Ein/Aus
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  /// Nicht verwendet, keine GA's in MFX
  fn get_ga_tel(&self, _adr: usize, _port: usize, _value: bool, _ddl_tel: &mut DdlTel) {
    assert!(false, "MFX unterstützt keine GA, Aufruf get_ga_tel");
  }

  /// Liefert das Idle Telegramm dieses Protokolles
  /// Return None wenn kein Idle Telegramm vorhanden ist
  /// MFX hat kein Idle Telegramm. Als Idle wird UID der Zentrale und Dekoder Abfrage gesendet
  /// Sobald eine GL vorhanden ist, wird keine Idle mehr gesendet, UID Zentrale wird dann periodisch
  /// über get_protokoll_telegrammme im Intervall INTERVALL_UID gesendet
  fn get_idle_tel(&mut self) -> Option<DdlTel> {
    let mut ddl_tel = self.get_gl_new_tel(0, false);
    self.send_uid_regcounter(&mut ddl_tel);
    Some(ddl_tel)
  }
  /// Liefert zusätzliche, Protokoll spezifische Telegramme (z.B. bei MFX die UID & Neuanmeldezähler der Zentrale)
  /// Liefert None, wenn es nichts zur versenden gibt
  /// Hier für MFX wird periodisch die UID / Neuanmeldezähler der Zentrale versandt
  fn get_protokoll_telegrammme(&mut self) -> Option<DdlTel> {
    let now = Instant::now();
    if now >= (self.zeitpunkt_uid + INTERVALL_UID) {
      self.zeitpunkt_uid = now;
      self.get_idle_tel()
    } else {
      None
    }
  }
}
