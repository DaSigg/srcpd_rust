use std::{
  collections::HashMap,
  sync::mpsc::{self, Receiver, Sender},
  thread,
  time::Duration,
};

use gpio_cdev::LineHandle;
use log::debug;

use crate::{
  srcp_dcc_prog::{DccCvTel, DccCvTelType, DccProgThread, DCC_SM_TYPE_CV, DCC_SM_TYPE_CVBIT},
  srcp_protocol_ddl::{DdlProtokoll, DdlTel, GLDriveMode, SmReadWrite},
};

//SPI Baudrate für DCC/NMRA.
//Diese wird so gewählt, dass ein kurzer 58/58us Impuls (logisch 1) in einem Byte (0xF0) ausgegeben werden kann,
//ein langer 116/116us Impuls wird dann als 2 Bytes (0xFF, 0x00) ausgegeben.
//Damit hätten wird für 8 Bit also 116us -> 1 Bit 14.5us -> 68966 Baud.
//Damit werden NMRA/DCC Pakete inkl. der führenden Sync. Bytes leider nicht immer >= 96 Bytes -> DMA Mode und keine Pause nach 8 Bits
//Hinweis: kleinst möglicher SPI Clock bei 250 MHz core_freq ist 30.5 kHz.
const SPI_BAUDRATE_NMRA: u32 = 68966;
//Deshalb wird auch hier die doppelte Baudrate verwendet und dann wie folgt kodiert:
//1: 0xFF, 0x00
//0: 0xFF, 0xFF, 0x00, 0x00
const SPI_BAUDRATE_NMRA_2: u32 = SPI_BAUDRATE_NMRA * 2;

static DCC_BIT_1: &'static [u8] = &[0xFF, 0x00]; //1
static DCC_BIT_0: &'static [u8] = &[0xFF, 0xFF, 0x00, 0x00]; //0

/// Max. erlaubte GL Kurz-Adresse (V1, 7 Bit)
const MAX_DCC_GL_ADRESSE_KURZ: u32 = 127;
/// Max. erlaubte GL Lang-Adresse (V2, 14 Bit)
const MAX_DCC_GL_ADRESSE_LANG: u32 = 10239;
/// Max. erlaubte GA Adresse gem. NMRA S-9.2.1 Dekoder mit 4*2 Ausgängen, 9 Bit für Dekoder Adresse, 2 Bit Adr auf Dekoder.
/// Dekoderadresse 0x1FF mit Subadr 3 (alle Adr Bits = 1) ist reserviert für E-Stop, es bleiben also inkl. Adr 1 bis 2047
const MAX_DCC_GA_ADRESSE: u32 = 2047;
/// Anzahl sync. Bits
const ANZ_DCC_SYNC: usize = 16;
/// Anzahl sync. Bits Prog Gleis
const ANZ_DCC_SYNC_PROG_GLEIS: usize = 25;
/// Verzögerung zwischen zwei Frames an selbe Adresse zwischen Stop- und Startbit ist 5ms.
/// Das Sync. Muster am Anfang darf hier noch abgezählt werden.
const DCC_DELAY_GLEICHE_ADR: Duration = Duration::from_millis(4);

/// Max. Tel. Längen, Annahme: alle variablen Bits sind 0
/// 1 Byte: 1111111111111111 0 00000000 0 00000000 1 -> 17*1, 18*0
/// 2 Byte: 1111111111111111 0 00000000 0 00000000 0 00000000 1 -> 17*1, 27*0
/// 3 Byte: 1111111111111111 0 00000000 0 00000000 0 00000000 0 00000000 1 -> 17*1, 36*0
const DCC_MAX_LEN_BASIS: usize = 17 * 2 + 8 * 4; //Sync mit xor und Schlussbit
const DCC_MAX_LEN_PRO_BYTE: usize = 9 * 4;

//DCC Instruktionen erstes Datenbyte. Die 3 MSB Bits sind relevant
///DCC Advanced Operations
const DCC_INST_ADVOP: u8 = 0b00100000;
///DCC Advanced Operations -> 128 Speed controll (die unteren 5 Bit zu DCC_INST_ADVOP)
const DCC_INST_ADVOP_128_SPEED: u8 = 0b00011111;
///DCC Instr. Drive Reverse
const DCC_INST_DRIVE_REVERSE: u8 = 0b01000000;
///DCC Instr. Drive Forward
const DCC_INST_DRIVE_FORWARD: u8 = 0b01100000;
///DCC Instr. F0-F4
const DCC_INST_F0_F4: u8 = 0b10000000;
const BIT_MASK_F0_F4: u64 = 0b11111;
///DCC Instr. F5-F12
const DCC_INST_F5_F12: u8 = 0b10100000;
///DCC Instr. F5-F12 für F5-F8
const DCC_INST_F5_F8: u8 = DCC_INST_F5_F12 | 0b00010000;
const BIT_MASK_F5_F8: u64 = 0b111100000;
///DCC Instr. F5-F12 für F9-F12
const DCC_INST_F9_F12: u8 = DCC_INST_F5_F12 | 0b00000000;
const BIT_MASK_F9_F12: u64 = 0b1111000000000;
///DCC Instr. Expansion
const DCC_INST_EXP: u8 = 0b11000000;
///DCC Instr. Expansion -> F13-F20 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F13_F20: u8 = DCC_INST_EXP | 0b00011110;
const BIT_MASK_F13_F20: u64 = 0b111111110000000000000;
///DCC Instr. Expansion -> F21-F28 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F21_F28: u8 = DCC_INST_EXP | 0b00011111;
const BIT_MASK_F21_F28: u64 = 0b11111111000000000000000000000;
///DCC Instr. Expansion -> F29-F36 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F29_F36: u8 = DCC_INST_EXP | 0b00011000;
const BIT_MASK_F29_F36: u64 = 0b1111111100000000000000000000000000000;
///DCC Instr. Expansion -> F37-F44 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F37_F44: u8 = DCC_INST_EXP | 0b00011001;
const BIT_MASK_F37_F44: u64 = 0b111111110000000000000000000000000000000000000;
///DCC Instr. Expansion -> F45-F52 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F45_F52: u8 = DCC_INST_EXP | 0b00011010;
const BIT_MASK_F45_F52: u64 = 0b11111111000000000000000000000000000000000000000000000;
///DCC Instr. Expansion -> F53-F60 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F53_F60: u8 = DCC_INST_EXP | 0b00011011;
const BIT_MASK_F53_F60: u64 = 0b1111111100000000000000000000000000000000000000000000000000000;
///DCC Instr. Expansion -> F61-F68 (die unteren 5 Bit zu DCC_INST_EXP)
const DCC_INST_EXP_F61_F68: u8 = DCC_INST_EXP | 0b00011100;
const BIT_MASK_F61_F63: u64 = 0b1110000000000000000000000000000000000000000000000000000000000000;

///DCC Instr. für Programmiermodus
///1. Byte, Kennnung Programmierung Hauptgleis, oberen 4 Bits
const DCC_PROG_HAUPT_GLEIS: u8 = 0b11100000;
///1. Byte, Kennnung Programmierung Proggleis, oberen 4 Bits
const DCC_PROG_PROG_GLEIS: u8 = 0b01110000;
///1. Byte, Kennnung Ver Byte, Bit 2 & 3
const DCC_PROG_KK_VER_BYTE: u8 = 0b00000100;
///1. Byte, Kennnung Write Byte, Bit 2 & 3
const DCC_PROG_KK_WRITE_BYTE: u8 = 0b00001100;
///1. Byte, Kennnung Bit, Bit 2 & 3
const DCC_PROG_KK_BIT: u8 = 0b00001000;

//Grenzen für Speed Steps bis und mit diesem Wert
const SPEED_STEP_4BIT: usize = 14;
const SPEED_STEP_5BIT: usize = 29;

pub enum DccVersion {
  V1, //Kurze Lokadresse bis 127
  V2, //Lange Lokadresse 128 bis 10239. Bis 127 wird gemäss DCC Standard automatisch auch hier immer mit der kurzen Adresse gearbeitet
}

pub struct DccProtokoll {
  /// Version 1 oder 2, Keine Unterschiede für GA, nur kurze Lange GL Adr.
  version: DccVersion,
  /// Halten Richtung bei Richtung Nothalt
  old_drive_mode: [GLDriveMode; MAX_DCC_GL_ADRESSE_LANG as usize + 1],
  /// Erkennung Funktionswechsel für die nicht immer gesendeten höheren Fx
  old_funktionen: [u64; MAX_DCC_GL_ADRESSE_LANG as usize + 1],
  /// Anzahl Initialisierte Funktionen
  funk_anz: [usize; MAX_DCC_GL_ADRESSE_LANG as usize + 1],
  /// Ist SM Mode auf diesem Protokoll aktiviert?
  sm_aktiv: bool,
  /// Channel für Aufträge an Prog Thread
  tx_to_prog: Sender<SmReadWrite>,
  /// Channel für Antworten von Prog Thread von SM Read/Write
  rx_from_prog_read_write_cv: Receiver<SmReadWrite>,
  /// Channel für Tel. Sendeaufträge vom Prog Thread
  rx_tel_from_prog: Receiver<DccCvTel>,
}

impl DccProtokoll {
  /// Neue Instanz erstellen
  /// # Arguments
  /// * version - V1 oder V2
  /// * ack_line_handle - GPIO Handle über das der Programmier ACK Impuls eingelesen werden kann.
  pub fn from(version: DccVersion, ack_line_handle: &'static LineHandle) -> DccProtokoll {
    //Channels zur Kommunikation mit Prog Thread
    //-> Aufträge zum Prog Thread
    let (tx_to_prog, rx_in_prog): (Sender<SmReadWrite>, Receiver<SmReadWrite>) = mpsc::channel();
    //<- Antworten ReadCV/WriteCV vom Prog Thread
    let (tx_from_prog_read_write_cv, rx_from_prog_read_write_cv): (
      Sender<SmReadWrite>,
      Receiver<SmReadWrite>,
    ) = mpsc::channel();
    //<- DCC Tel. Sendeaufträge vom Prog Thread
    let (tx_tel_from_prog, rx_tel_from_prog): (Sender<DccCvTel>, Receiver<DccCvTel>) =
      mpsc::channel();
    //DCC Programmier Servicemode Thread starten
    thread::Builder::new()
      .name("DCC Prog Thread".to_string())
      .spawn(move || {
        DccProgThread::new(
          rx_in_prog,
          tx_from_prog_read_write_cv,
          tx_tel_from_prog,
          ack_line_handle,
        )
        .execute()
      })
      .unwrap();
    DccProtokoll {
      version,
      old_drive_mode: [GLDriveMode::Vorwaerts; MAX_DCC_GL_ADRESSE_LANG as usize + 1],
      old_funktionen: [0; MAX_DCC_GL_ADRESSE_LANG as usize + 1],
      funk_anz: [0; MAX_DCC_GL_ADRESSE_LANG as usize + 1],
      sm_aktiv: false,
      tx_to_prog,
      rx_from_prog_read_write_cv,
      rx_tel_from_prog,
    }
  }

  /// Fügt Start Syncmuster 16*1 mit abschliessendem 0 zum letzten in ddl_tel enthaltenen Telegramm
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem Sync. ergänzt werden soll
  /// * prog_gleis - false: Sync. für normale Telegramme, true: doppeelte Anzahl Syncs für Prog. Gleis
  fn add_sync(&self, ddl_tel: &mut DdlTel, prog_gleis: bool) {
    for _ in 0..if prog_gleis {
      ANZ_DCC_SYNC_PROG_GLEIS
    } else {
      ANZ_DCC_SYNC
    } {
      ddl_tel
        .daten
        .last_mut()
        .unwrap()
        .extend_from_slice(DCC_BIT_1);
    }
    ddl_tel
      .daten
      .last_mut()
      .unwrap()
      .extend_from_slice(DCC_BIT_0);
  }

  /// Fügt ein Byte zum DDL DCC Telegramm hinzu und aktualisiert die Prüfsumme (exor)
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem ein Byte (MSB zuerst) hinzugefügt werden soll
  /// * value - Byte, das hinzugefügt werden soll
  /// * xor - Alter und nachher aktueller xor Wert
  /// * endbit - false: 0 Bit als Byteendemarke einfügen, true 1 als Byteendemarke einfügen für Telegrammende
  fn add_byte(&self, ddl_tel: &mut DdlTel, value: u8, xor: &mut u8, endbit: bool) {
    for i in (0..8).rev() {
      //Geht von 7 bis 0
      ddl_tel
        .daten
        .last_mut()
        .unwrap()
        .extend_from_slice(if (value & (1 << i)) == 0 {
          DCC_BIT_0
        } else {
          DCC_BIT_1
        });
    }
    *xor ^= value;
    //Byteendemarke, normalerweise 0, bei Telegrammende 1
    ddl_tel
      .daten
      .last_mut()
      .unwrap()
      .extend_from_slice(if endbit { DCC_BIT_1 } else { DCC_BIT_0 });
  }
  /// Fügt die Addresse (1 Byte bis 127, 2 Byte wenn grösser) mit abschliessendem 0 zum letzten in
  /// ddl_tel enthaltenen Tel. hinzu
  /// Liefert die aktuelle Prüfsumme (exor) nach Adresse zurück
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem die Adresse ergänzt werden soll
  /// * adr - Die Adresse
  fn add_adr(&self, ddl_tel: &mut DdlTel, adr: u32) -> u8 {
    let mut xor: u8 = 0;
    if adr <= MAX_DCC_GL_ADRESSE_KURZ {
      self.add_byte(ddl_tel, (adr & 0xFF).try_into().unwrap(), &mut xor, false);
    } else {
      //14 Bit Adr. Es kommen 6 bit MSB ins erste byte, dann 8 Bit bis zum LSB
      //Start des ersten Adr. Bytes mit 11
      let adr_msb: u8 = (0b11000000 | ((adr >> 8) & 0xFF)).try_into().unwrap();
      //Dann die 6 Bits ab MSB, mit MAX_DCC_GL_ADRESSE_LANG ist sichergestellt, dass die ersten 6 Bits
      //nur von 000000 bis 100111 gehen können
      self.add_byte(ddl_tel, adr_msb, &mut xor, false);
      //Und noch ein byte mit den 8 LSB Bits
      self.add_byte(ddl_tel, (adr & 0xFF).try_into().unwrap(), &mut xor, false);
    }
    xor
  }

  ///DCC Telegramm abschliessen.
  /// - xor
  /// - Doppelte 1 als Abschluss um sicher eine 1 gültig (letzte Flanke) zu haben
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem die XOR Checksumme ergänzt werden soll
  /// * xor - xor Prüfsumme
  fn add_xor(&self, ddl_tel: &mut DdlTel, mut xor: u8) {
    //Checksumme ergänzen
    self.add_byte(ddl_tel, xor, &mut xor, true);
    //Und nochmals ein 1 Bit damit noch ein korrekter Abschluss (letzte Flanke) da ist
    ddl_tel
      .daten
      .last_mut()
      .unwrap()
      .extend_from_slice(DCC_BIT_1);
  }
  /// Telegramm für 8 Funktionen aus dem Bereich F13 bis F68 erzeugen und hinzufügen wenn sich
  /// eine Funktion der Gruppe geändert hat oder Refresh verlangt wird.
  /// # Arguments
  /// * ddl_tel - Telegramm, bei dem die Adresse ergänzt werden soll
  /// * adr - Adresse
  /// * funktionen - Alle Funktionszustände
  /// * mask - Bitmaske für die relevanten Funktionen
  /// * shift - Anzahl bits die fun nach rechts geschoben werden muss
  /// * ddl_cmd - Das zu setzende DDL Kommandobyte
  /// * refersh - true wenn Refreshkommando, telegramm wird auch erzeugt wenn keine Veränderung
  fn add_f13_f68(
    &self, ddl_tel: &mut DdlTel, adr: u32, funktionen: u64, mask: u64, shift: usize, ddl_cmd: u8,
    refresh: bool,
  ) {
    //Auf Veränderungen prüfen
    if (((self.old_funktionen[adr as usize] ^ funktionen) & mask) != 0) || refresh {
      //Worst case Länge: 2 Bytes Adresse + 2 Nutzbytes
      ddl_tel.daten.push(Vec::with_capacity(
        DCC_MAX_LEN_BASIS + 4 * DCC_MAX_LEN_PRO_BYTE,
      ));
      self.add_sync(ddl_tel, false);
      let mut xor = self.add_adr(ddl_tel, adr);
      self.add_byte(ddl_tel, ddl_cmd, &mut xor, false);
      let f = <u64 as TryInto<u8>>::try_into((funktionen & mask) >> shift).unwrap();
      self.add_byte(ddl_tel, f, &mut xor, false);
      self.add_xor(ddl_tel, xor);
    }
  }
  /// Liefert ein DCC CV Read/Write Telegramm.
  /// # Arguments
  /// * cvtel - Zu erzeugendes Telegramm
  fn get_cv_tel(&mut self, cvtel: &DccCvTel) -> DdlTel {
    debug!("DCC get_cv_tel {:?}", cvtel);
    //CV's gehen von 1 bis 1024, im Telegramm mit 10 Bit von 0 bis 1023
    let cv = cvtel.cv - 1;
    //GL Tel. als Basis, Refresh = 1 mal senden.
    //Die ür Write 2 oder 5 mal OHNE JEDE Pause gesendet werden muss, kann nicht mit Wiederholungen gearbeitet werden,
    //da dabei immer eine kurze Pause entsteht. Es werden alle Daten kopiert.
    let mut tel = self.get_gl_new_tel(cvtel.adr, true, cvtel.trigger);
    //Telegramme müssen direkt aufeinander folgen
    tel.delay = Duration::ZERO;
    match cvtel.dcc_cv_type {
      DccCvTelType::VerifyBit(val, bitnr) | DccCvTelType::WriteBit(val, bitnr, _) => {
        //Hauptgleisprog. nur bei Write ohne Prog Gleis, alles andere -> Prog Gleis
        let haupt_gleis = matches!(cvtel.dcc_cv_type, DccCvTelType::WriteBit(_, _, false));
        let write = matches!(cvtel.dcc_cv_type, DccCvTelType::WriteBit(_, _, _));
        let mut xor: u8 = 0;
        self.add_sync(&mut tel, !haupt_gleis);
        //Tel. Format Hauptgleis: Adr - 1110-KKVV VVVV-VVVV 111K-DBBB
        //Tel. Format Proggleis: 0111-KKVV VVVV-VVVV 111K-DBBB
        //Im ersten Byte KK und an Bit Pos 0 & 1 CV Bit 8 und 9
        let mut prog_byte_1 = ((cv >> 8) & 0b00000011) as u8 | DCC_PROG_KK_BIT;
        if haupt_gleis {
          //Adresse, nur für Hauptgleis Programmierung
          xor = self.add_adr(&mut tel, cvtel.adr);
          prog_byte_1 |= DCC_PROG_HAUPT_GLEIS;
        } else {
          prog_byte_1 |= DCC_PROG_PROG_GLEIS;
        }
        self.add_byte(&mut tel, prog_byte_1, &mut xor, false);
        //Unter 8 Bit CV
        self.add_byte(&mut tel, (cv & 0xFF) as u8, &mut xor, false);
        //Letztes Byte 111K-DBBB, K=0=Verify / K=1=Write, D=BitValue, BBB=Bitnr
        self.add_byte(
          &mut tel,
          0b11100000
            | if write { 0b00010000 } else { 0b00000000 }
            | if val { 0b00001000 } else { 0b00000000 }
            | (bitnr & 0b00000111),
          &mut xor,
          false,
        );
        //XOR
        self.add_xor(&mut tel, xor);
        //CV Write Telegramme auf Prog Gleis MÜSSEN 5 mal, bei Hauptgleis 2 mal hintereinander gesendet werden
        let daten_1_tel = tel.daten.last().unwrap().clone();
        for _ in 1..if haupt_gleis { 2 } else { 5 } {
          tel
            .daten
            .last_mut()
            .unwrap()
            .extend_from_slice(daten_1_tel.as_slice());
        }
      }
      DccCvTelType::VerifyByte(val) | DccCvTelType::WriteByte(val, _) => {
        //Hauptgleisprog. nur bei Write ohne Prog Gleis, alles andere -> Prog Gleis
        let haupt_gleis = matches!(cvtel.dcc_cv_type, DccCvTelType::WriteByte(_, false));
        let write = matches!(cvtel.dcc_cv_type, DccCvTelType::WriteByte(_, _));
        let mut xor: u8 = 0;
        self.add_sync(&mut tel, !haupt_gleis);
        //Tel. Format Hauptgleis: Adr - 1110-KKVV VVVV-VVVV DDDD-DDDD
        //Tel. Format Proggleis: 0111-KKVV VVVV-VVVV DDDD-DDDD
        //Im ersten Byte KK und an Bit Pos 0 & 1 CV Bit 8 und 9
        let mut prog_byte_1 = ((cv >> 8) & 0b00000011) as u8
          | if write {
            DCC_PROG_KK_WRITE_BYTE
          } else {
            DCC_PROG_KK_VER_BYTE
          };
        if haupt_gleis {
          //Adresse, nur für Hauptgleis Programmierung
          xor = self.add_adr(&mut tel, cvtel.adr);
          prog_byte_1 |= DCC_PROG_HAUPT_GLEIS;
        } else {
          prog_byte_1 |= DCC_PROG_PROG_GLEIS;
        }
        self.add_byte(&mut tel, prog_byte_1, &mut xor, false);
        //Unter 8 Bit CV
        self.add_byte(&mut tel, (cv & 0xFF) as u8, &mut xor, false);
        //Value
        self.add_byte(&mut tel, val, &mut xor, false);
        //XOR
        self.add_xor(&mut tel, xor);
        //CV Write Telegramme auf Prog Gleis MÜSSEN 5 mal, bei Hauptgleis 2 mal hintereinander gesendet werden
        let daten_1_tel = tel.daten.last().unwrap().clone();
        for _ in 1..if haupt_gleis { 2 } else { 5 } {
          tel
            .daten
            .last_mut()
            .unwrap()
            .extend_from_slice(daten_1_tel.as_slice());
        }
      }
    }
    tel
  }
}
impl DdlProtokoll for DccProtokoll {
  /// GL Init Daten setzen. Welche Daten verwendet werden ist Protokollabhängig.
  /// Liefert immer None, kein GL Init Tel. notwendig
  /// # Arguments
  /// * adr - Adresse der Lok
  /// * uid - nicht verwendet
  /// * funk_anz - Anzahl tatsächlich verwendete Funktionen. Kann, je nach Protokoll, dazu
  ///              verwendet werden, nur Telegramme der verwendeten Funktionen zu senden.
  /// * power - nicht verwendet
  /// * trigger - Oszi Trigger bei Ausgabe?
  fn init_gl(
    &mut self, adr: u32, _uid: Option<u32>, funk_anz: usize, _power: bool, _trigger: bool,
  ) -> Option<DdlTel> {
    self.funk_anz[adr as usize] = funk_anz;
    None
  }
  /// Liefert die max. erlaubte Lokadresse
  fn get_gl_max_adr(&self) -> u32 {
    match self.version {
      DccVersion::V1 => MAX_DCC_GL_ADRESSE_KURZ,
      DccVersion::V2 => MAX_DCC_GL_ADRESSE_LANG,
    }
  }
  /// Wieviele Speedsteps werden vom Protokoll unterstützt
  /// Maximale Möglichkeit DCC: 127
  fn get_gl_max_speed_steps(&self) -> usize {
    127
  }
  /// Liefert die max. erlaubte Schaltmoduladdresse
  fn get_ga_max_adr(&self) -> u32 {
    MAX_DCC_GA_ADRESSE
  }
  /// Liefert die max. Anzahl der unterstützten Funktionen
  fn get_gl_anz_f(&self) -> usize {
    64 //Eigentlich kann DCC Total 69 (F0-F68), im Moment reicht mir das aber, die Funktionen werden in ganz srcp_rust in einem u64 verwaltet
  }
  /// Liefert die Anzahl Funktionen (inkl. F0) die im Basistelegramm enthalten sind
  /// Muss immer <= "get_Anz_F" sein.
  fn get_gl_anz_f_basis(&self) -> usize {
    //F0 bis F5 sind 1 Byte Kommandos, Zustand wird direkt als Bits gesendet. Diese nehmen wir zur Basis die immer gesendet werden.
    5
  }
  /// Liefert ein leeres GL Telegramm zur Verwendung in "get_gl_basis_tel" und / oder "get_gl_zusatz_tel".
  /// Als Initiale Kapazitäte wird von einem 4 Byte (lange Adresse plus 2 Nutzbytes) DCC Telegramm ausgegangen.
  /// # Arguments
  /// * adr - Adresse der Lok, keine Verwendunbg, nur Debug Support
  /// * refresh - Wenn true: Aufruf aus Refres Cycle, einmalige Telegramm Versendung,
  ///             Wenn false: Aufruf wegen neuem Lokkommando, mehrmaliges Versenden
  /// * trigger - Oszi Trigger bei Ausgabe?
  fn get_gl_new_tel(&mut self, adr: u32, refresh: bool, trigger: bool) -> DdlTel {
    DdlTel::new(
      adr,
      SPI_BAUDRATE_NMRA_2,
      DCC_DELAY_GLEICHE_ADR,
      false,
      DCC_MAX_LEN_BASIS + 4 * DCC_MAX_LEN_PRO_BYTE,
      if refresh { 1 } else { 2 }, //Neue Lokkommandos werden immer 2-fach gesendet
      trigger,
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
  /// * funktionen - Die gewünschten Funktionen, berücksichtigt bis "get_Anz_F_Basis"
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_gl_basis_tel(
    &mut self, adr: u32, drive_mode: GLDriveMode, speed: usize, speed_steps: usize,
    funktionen: u64, ddl_tel: &mut DdlTel,
  ) {
    //Prüfung Gültigkeit Adresse
    assert!(adr <= self.get_gl_max_adr(), "DCC GL Adresse zu gross");
    //Gültigkeit speed prüfen
    assert!(speed <= speed_steps, "DCC Speed > Speed Steps");
    //Drivemode für Richtung, wenn Nothalt dann der letzte
    let drive_mode_used: GLDriveMode = if drive_mode == GLDriveMode::Nothalt {
      self.old_drive_mode[adr as usize]
    } else {
      drive_mode
    };
    self.old_drive_mode[adr as usize] = drive_mode_used;
    //Speed 1 = Nothalt
    let speed_used = if drive_mode == GLDriveMode::Nothalt {
      1
    } else {
      if speed == 0 {
        0
      } else {
        speed + 1 //Und es fängt dann mit 2 an, bei 14 Steps ist max. 15, bei 28 Steps 29
      }
    };
    self.add_sync(ddl_tel, false);
    //Addresse in 1 oder 2 Bytes
    let mut xor = self.add_adr(ddl_tel, adr);
    //Kommando ist nun abhängig von den gewünschten Anzahl Speed Steps
    if speed_steps > SPEED_STEP_5BIT {
      //Kommando Speed mit 128 Steps und extra Byte mit Speed
      self.add_byte(
        ddl_tel,
        DCC_INST_ADVOP | DCC_INST_ADVOP_128_SPEED,
        &mut xor,
        false,
      );
      //MSB Richtung und 7 Bit Speed
      let speed_byte: u8 = ((speed & 0b01111111)
        | if drive_mode_used == GLDriveMode::Vorwaerts {
          0b10000000
        } else {
          0b00000000
        })
      .try_into()
      .unwrap();
      self.add_byte(ddl_tel, speed_byte, &mut xor, false);
    } else {
      let mut speed_byte: u8 = if drive_mode_used == GLDriveMode::Vorwaerts {
        DCC_INST_DRIVE_FORWARD
      } else {
        DCC_INST_DRIVE_REVERSE
      };
      if speed_steps > SPEED_STEP_4BIT {
        //Kommando Fahren vorwärts oder rückwärts plus 5 Bit Speed
        //Bit0-3 -> Bit 1-4 Speed, Bit4 -> Bit 0 Speed
        //Bit4 ist "Zwischenschritt", Nothalt aber trotzdem 1 in Bit0-3, also eigentlich Speed 2 ... :-(
        let speed_used_5bit = match speed_used {
          0 => 0,
          1 => 2,              //Nothalt
          _ => speed_used + 2, //speed_used 2..29, geht damit von 4 bis 31, auch 3 wird noch als Nothalt interpretiert
        };
        speed_byte |= TryInto::<u8>::try_into((speed_used_5bit >> 1) & 0b00001111).unwrap()
          | TryInto::<u8>::try_into((speed_used_5bit << 4) & 0b00010000).unwrap();
      } else {
        //Kommando Fahren vorwärts oder rückwärts plus 4 Bit Speed
        //Bit0-3 -> Bit 0-3 Speed
        //Bit 4 ist F0
        speed_byte |= TryInto::<u8>::try_into(speed_used & 0b00001111).unwrap()
          | TryInto::<u8>::try_into(((funktionen & 1) << 4) & 0b00010000).unwrap();
      }
      self.add_byte(ddl_tel, speed_byte, &mut xor, false);
    }
    self.add_xor(ddl_tel, xor);

    //Nur wenn notwendig: F0..F4 Telegramm
    //Je nach Speedsteps muss F0 hier berücksichtigt werden oder nicht
    if ((self.funk_anz[adr as usize] > 0) && (speed_steps > SPEED_STEP_4BIT))
      || ((self.funk_anz[adr as usize] > 1) && (speed_steps <= SPEED_STEP_4BIT))
    {
      //Und nun noch F0 bis F5. Da 5ms Pause zwischen 2 DCC Telegrammen an selbe Adresse notwendig ist,
      //als 2. unabhängigs Telegramm.
      //Worst case Länge: 2 Bytes Adresse + 1 Nutzbyte
      ddl_tel.daten.push(Vec::with_capacity(
        DCC_MAX_LEN_BASIS + 3 * DCC_MAX_LEN_PRO_BYTE,
      ));
      self.add_sync(ddl_tel, false);
      //Addresse in 1 oder 2 Bytes
      xor = self.add_adr(ddl_tel, adr);
      //DCC_INST_F0_F4 -> Bit 0..3 = F1..F4, Bit4 = F0
      //Falls nur 4 Bit Speed, dann wurde F0 bereits mit Speed Kommando übertragen.
      //Macht aber nichts, wenn F0 immer hier auch noch übertragen wird.
      let mut f0_f4_byte = DCC_INST_F0_F4;
      f0_f4_byte |= <u64 as TryInto<u8>>::try_into((funktionen & BIT_MASK_F0_F4) >> 1).unwrap();
      if (funktionen & 1) != 0 {
        f0_f4_byte |= 0b00010000;
      }
      self.add_byte(ddl_tel, f0_f4_byte, &mut xor, false);
      self.add_xor(ddl_tel, xor);
    }
    //F0..F4 übernehmen
    self.old_funktionen[adr as usize] &= !BIT_MASK_F0_F4;
    self.old_funktionen[adr as usize] |= funktionen & BIT_MASK_F0_F4;
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
    let funk_anz = self.funk_anz[adr as usize];
    //F5..F8 auf Veränderungen prüfen
    if ((((self.old_funktionen[adr as usize] ^ funktionen) & BIT_MASK_F5_F8) != 0) || refresh)
      && (funk_anz > 5)
    {
      //Worst case Länge: 2 Bytes Adresse + 1 Nutzbyte
      ddl_tel.daten.push(Vec::with_capacity(
        DCC_MAX_LEN_BASIS + 3 * DCC_MAX_LEN_PRO_BYTE,
      ));
      self.add_sync(ddl_tel, false);
      let mut xor = self.add_adr(ddl_tel, adr);
      let mut f5_f8_byte = DCC_INST_F5_F8;
      f5_f8_byte |= <u64 as TryInto<u8>>::try_into((funktionen & BIT_MASK_F5_F8) >> 5).unwrap();
      self.add_byte(ddl_tel, f5_f8_byte, &mut xor, false);
      self.add_xor(ddl_tel, xor);
    }
    //F9..F12 auf Veränderungen prüfen
    if ((((self.old_funktionen[adr as usize] ^ funktionen) & BIT_MASK_F9_F12) != 0) || refresh)
      && (funk_anz > 9)
    {
      //Worst case Länge: 2 Bytes Adresse + 1 Nutzbyte
      ddl_tel.daten.push(Vec::with_capacity(
        DCC_MAX_LEN_BASIS + 3 * DCC_MAX_LEN_PRO_BYTE,
      ));
      self.add_sync(ddl_tel, false);
      let mut xor = self.add_adr(ddl_tel, adr);
      let mut f9_f12_byte = DCC_INST_F9_F12;
      f9_f12_byte |= <u64 as TryInto<u8>>::try_into((funktionen & BIT_MASK_F9_F12) >> 9).unwrap();
      self.add_byte(ddl_tel, f9_f12_byte, &mut xor, false);
      self.add_xor(ddl_tel, xor);
    }
    if funk_anz > 13 {
      self.add_f13_f68(
        ddl_tel,
        adr,
        funktionen,
        BIT_MASK_F13_F20,
        13,
        DCC_INST_EXP_F13_F20,
        refresh,
      );
    }
    if funk_anz > 21 {
      self.add_f13_f68(
        ddl_tel,
        adr,
        funktionen,
        BIT_MASK_F21_F28,
        21,
        DCC_INST_EXP_F21_F28,
        refresh,
      );
    }
    if funk_anz > 29 {
      self.add_f13_f68(
        ddl_tel,
        adr,
        funktionen,
        BIT_MASK_F29_F36,
        29,
        DCC_INST_EXP_F29_F36,
        refresh,
      );
    }
    if funk_anz > 37 {
      self.add_f13_f68(
        ddl_tel,
        adr,
        funktionen,
        BIT_MASK_F37_F44,
        37,
        DCC_INST_EXP_F37_F44,
        refresh,
      );
    }
    if funk_anz > 45 {
      self.add_f13_f68(
        ddl_tel,
        adr,
        funktionen,
        BIT_MASK_F45_F52,
        45,
        DCC_INST_EXP_F45_F52,
        refresh,
      );
    }
    if funk_anz > 53 {
      self.add_f13_f68(
        ddl_tel,
        adr,
        funktionen,
        BIT_MASK_F53_F60,
        53,
        DCC_INST_EXP_F53_F60,
        refresh,
      );
    }
    if funk_anz > 61 {
      self.add_f13_f68(
        ddl_tel,
        adr,
        funktionen,
        BIT_MASK_F61_F63,
        61,
        DCC_INST_EXP_F61_F68,
        refresh,
      );
    }
    //Alles ab F5 übernehmen
    self.old_funktionen[adr as usize] &= 0b11111;
    self.old_funktionen[adr as usize] |= funktionen & !0b11111;
  }
  /// Liefert ein leeres GA Telegramm zur Verwendung in "get_ga_tel".
  /// # Arguments
  /// * adr - Adresse GA, keine Verwendunbg, nur Debug Support
  /// * trigger - Oszi Trigger?
  fn get_ga_new_tel(&self, adr: u32, trigger: bool) -> DdlTel {
    DdlTel::new(
      adr,
      SPI_BAUDRATE_NMRA_2,
      DCC_DELAY_GLEICHE_ADR,
      false,
      DCC_MAX_LEN_BASIS + 2 * DCC_MAX_LEN_PRO_BYTE,
      2, //GA wird immer nur bei Bedarf gesendet, kein Refresh. Deshalb immer 2-fach senden
      trigger,
    )
  }
  /// Erzeugt ein GA Telegramm
  /// # Arguments
  /// * adr - GA Adresse
  /// * port - Port auf dem Schaltdekoder 0 / 1
  /// * value - Gewünschter Zustand des Port Ein/Aus
  /// * ddl_tel - DDL Telegramm, bei dem des neue Telegramm hinzugefügt werden soll.
  fn get_ga_tel(&self, adr: u32, port: usize, value: bool, ddl_tel: &mut DdlTel) {
    self.add_sync(ddl_tel, false);
    let mut xor: u8 = 0;
    /* calculate the real address of the decoder and the pair number
     * of the switch. Definition, dass Useradr. 1-4 hier die Adresse 1 ist. Die Adr. 2044-2047 sind dann 0.*/
    let address = if adr < 2044 {(adr as usize - 1) / 4 + 1} else {0};
    let pairnr = if adr < 2044 {(adr as usize - 1) % 4} else {adr as usize % 4};
    /* address byte: 10AAAAAA (lower 6 bits) */
    self.add_byte(
      ddl_tel,
      (0b10000000 | (address & 0b00111111)).try_into().unwrap(),
      &mut xor,
      false,
    );
    /* address and data 1AAACDDO upper 3 address bits are inverted */
    /* C =  activate, DD = pairnr */
    self.add_byte(
      ddl_tel,
      (0b10000000
        | ((!address & 0b111000000) >> 2)
        | (if value { 0b00001000 } else { 0 })
        | (pairnr << 1)
        | (port & 0b00000001))
        .try_into()
        .unwrap(),
      &mut xor,
      false,
    );
    self.add_xor(ddl_tel, xor);
  }

  /// Liefert das Idle Telegramm dieses Protokolles
  /// Return None wenn kein Idle Telegramm vorhanden ist
  fn get_idle_tel(&mut self) -> Option<DdlTel> {
    //DCC Idle Telegramm: 1111111111111111 0 11111111 0 00000000 0 11111111 1
    let mut ddl_idle_tel = DdlTel::new(
      0,
      SPI_BAUDRATE_NMRA_2,
      Duration::ZERO, //Nicht notwendig für Idle Tel.
      false,
      DCC_MAX_LEN_BASIS + 2 * DCC_MAX_LEN_PRO_BYTE,
      1,
      false,
    );
    self.add_sync(&mut ddl_idle_tel, false);
    let mut xor: u8 = 0;
    self.add_byte(&mut ddl_idle_tel, 0b11111111, &mut xor, false);
    self.add_byte(&mut ddl_idle_tel, 0b00000000, &mut xor, false);
    //Checksumme ergänzen
    self.add_xor(&mut ddl_idle_tel, xor);
    Some(ddl_idle_tel)
  }

  /// Dekoderkonfiguration (SM) Start
  fn sm_init(&mut self) {
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
    self.tx_to_prog.send(sm_para.clone()).unwrap();
  }

  /// Liefert die Antwort sm_read_write zurück.
  /// None wenn keine Antwort verfügbar.
  fn sm_get_answer(&mut self) -> Option<SmReadWrite> {
    self.rx_from_prog_read_write_cv.try_recv().ok()
  }

  /// Liefert alle in "sm_read" und "sm_write" unterstützten Typen mit der Anzahl erwarteter Parameter
  /// ohne Value für SET.
  /// None wenn SM nicht unterstützt wird.
  fn sm_get_all_types(&self) -> Option<HashMap<String, usize>> {
    let mut result: HashMap<String, usize> = HashMap::new();
    //1 Parameter bei CV: CVNr
    //2 Parameter bei CVBIT: CVNr, BitNr
    result.insert(DCC_SM_TYPE_CV.to_string(), 1);
    result.insert(DCC_SM_TYPE_CVBIT.to_string(), 2);
    Some(result)
  }

  /// Liefert zusätzliche, Protokoll spezifische Telegramme (z.B. bei MFX die UID & Neuanmeldezähler der Zentrale)
  /// Liefert None, wenn es nichts zur versenden gibt
  /// Hier, wenn vorhanden, werden die CV Read/Write Telegramme erzeugt, wenn vom DCC Prog Thread verlangt.
  /// # Arguments
  /// * power : true wenn Power (Booster) ein, sonst false.
  ///           Normalerweise werden Telegramme nur bei Power On gesendet.
  ///           Ausnahme: SM DCC auf Prog. Gleis.
  ///           Hier nicht verwendet, CV Telegramme werden auf Prog. und Hauptgleis verwendet.
  fn get_protokoll_telegrammme(&mut self, _power: bool) -> Option<DdlTel> {
    let tel_from_prog = self.rx_tel_from_prog.try_recv();
    if let Ok(tel) = tel_from_prog {
      Some(self.get_cv_tel(&tel))
    } else {
      None
    }
  }
  /// Liefert das Idle Telegramm dieses Protokolles bei Power Off
  /// Return None wenn kein Idle Telegramm für Power Off vorhanden ist
  /// Wird hier für senden Rücksetzpaket bei aktiviertem SM verwendet.
  /// Damit wird bei Power Off im Leerlauf dauernd das Rücksetzpaket gesendet.
  fn get_idle_tel_power_off(&self) -> Option<DdlTel> {
    if self.sm_aktiv {
      //DCC Rücksetz Telegramm: 1111111111111111 0 00000000 0 00000000 0 00000000 1
      let mut ddl_reset_tel = DdlTel::new(
        0,
        SPI_BAUDRATE_NMRA_2,
        Duration::ZERO, //Nicht notwendig für Rücksetz Tel.
        false,
        DCC_MAX_LEN_BASIS + 2 * DCC_MAX_LEN_PRO_BYTE,
        1,
        false,
      );
      self.add_sync(&mut ddl_reset_tel, false);
      let mut xor: u8 = 0;
      self.add_byte(&mut ddl_reset_tel, 0b00000000, &mut xor, false);
      self.add_byte(&mut ddl_reset_tel, 0b00000000, &mut xor, false);
      //Checksumme ergänzen
      self.add_xor(&mut ddl_reset_tel, xor);
      Some(ddl_reset_tel)
    } else {
      //Nichts zu senden wenn kein SM aktiv ist
      None
    }
  }
}
