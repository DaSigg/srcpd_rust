use gpio::{sysfs::SysFsGpioInput, GpioIn, GpioValue};
use log::{debug, info, warn};
use std::{
  sync::mpsc::{Receiver, Sender},
  thread,
  time::{Duration, Instant},
};

/// Input RDS Qual Signal GPIO 23 (= Pin 16)
const GPIO_MFX_RDS_QAL: u16 = 23;
/// Input RDS Clk Signal GPIO 24 (= Pin 18)
const GPIO_MFX_RDS_CLK: u16 = 24;
/// Input RDS Clk Signal GPIO 25 (= Pin 22)
const GPIO_MFX_RDS_DAT: u16 = 24;

/// Zustände RDS Empfang
#[derive(PartialEq)]
enum StateRdsRx {
  StateStart1,   //Initiale 1 Folge
  StateStart010, //In Startkennung nach 1 Folge
  StateData,     //Nutzdatenempfang
  StateCheck,    //Prüfung, CRC
  StateFinal,
}

/// Thread zum Einlesen der MFX RDS Rückmeldungen.
/// Abarbeitung der Aufträge.
/// Wenn eine gültige Rückmeldung eingelesen wurde, dann wird sie über einen Channel
/// zurück zur MFX Protokollimplementierung gesendet ansonsten "None".
/// Es erfolgt immer eine Antwort auf eine Anfrage.
/// # Arguments
/// * rx - Empfang von Aufträge, es wird die erwartetet Anzahl Bytes übermittelt.
/// * tx - Sender zum versenden er eingelesen Rückmeldungen
fn mfx_rds_thread(rx: Receiver<usize>, tx: Sender<Option<Vec<u8>>>) {
  //GPIO's öffnen
  let mut gpio_mfx_rds_qal = SysFsGpioInput::open(GPIO_MFX_RDS_QAL)
    .expect(format!("GPIO_MFX_RDS_QAL konnte nicht geöffnet werden").as_str());
  let mut gpio_mfx_rds_clk = SysFsGpioInput::open(GPIO_MFX_RDS_CLK)
    .expect(format!("GPIO_MFX_RDS_CLK konnte nicht geöffnet werden").as_str());
  let mut gpio_mfx_rds_dat = SysFsGpioInput::open(GPIO_MFX_RDS_DAT)
    .expect(format!("GPIO_MFX_RDS_DAT konnte nicht geöffnet werden").as_str());

  loop {
    if let Ok(bytes_erwartet) = rx.recv() {
      //Es muss nun eine RDS Rückmeldung erfolgen.
      // - Warten bis RDS QUAL Meldung vorliegt
      // - Max. 23 mal 1, dann 010 (Startkennung)
      // - Anzahl erwartetet Datenbits
      // - 8 Bit Checksumme
      //Alles zusammen darf max. 200ms dauern, sonst wird abgebrochen
      let time_start = Instant::now();
      let mut state = StateRdsRx::StateStart1; //Warten auf QUAL und Data 1
      let mut count: usize = 0;
      //Ist ein Fehler aufgetreten? Es wird "None" als Ergebnis gesendet
      let mut result_error = false;
      //Antwort, Default "None"
      let mut result = None;
      let mut values = [0 as u8; 8];
      let mut rdsCheckSumme = 0 as u8;
      //RDS Antwort einlesen und verarbeiten
      while state != StateRdsRx::StateFinal {
        //Warte auf nächsten Clock, positive Flanke
        let mut clk_old = gpio_mfx_rds_clk.read_value().unwrap();
        loop {
          //Daten kommen etwa im 1ms Takt. Mit 200us Wartezeit sollte nichts verpasst werden
          thread::sleep(Duration::from_micros(200));
          let clk = gpio_mfx_rds_clk.read_value().unwrap();
          if (clk_old == GpioValue::Low) && (clk == GpioValue::High) {
            //Pos. Flanke erkannt
            break;
          }
          clk_old = clk;
          //ggf. Abbruch wegen Timeout
          if Instant::now() > (time_start + Duration::from_millis(200)) {
            warn!("MFX RDS thread Timeout.");
            result_error = true;
            break;
          }
        }
        if result_error {
          break;
        } else {
          match state {
            StateRdsRx::StateStart1 => {
              //Während Sync Sequenz sollte RDS Qual Meldung vorhandens ein.
              //Wenn nicht -> von vorne
              if gpio_mfx_rds_qal.read_value().unwrap() == GpioValue::Low {
                count = 0;
                info!("RDS Sync Abbruch. QUAL=0.");
              } else {
                if gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                  //Wieder ein 1 der Sync. Sequenz eingelesen
                  count += 1;
                } else {
                  //Von den 23 Bits 1 in der Sync. Sequenz will ich min. die letzten 3 gesetzt sehen, dann kann dieses 0 die Startsquenz sein
                  if (count >= 3) {
                    debug!("RDS new STATE_START010.");
                    state = StateRdsRx::StateStart010;
                  } else {
                    //Wieder von vorne beginnen
                    info!("RDS STATE_START1 Restart.");
                  }
                  count = 0;
                }
              }
            }
            StateRdsRx::StateStart010 => {
              //Erstes 0 wurde bereits gelesen, es wird noch 10 erwartet
              debug!("RDS STATE_START010 count={}.", count);
              if count == 0 {
                if gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                  //1 gelesen, alles OK
                  count += 1;
                } else {
                  //0, Falsch, Abbruch
                  count = 0;
                  state = StateRdsRx::StateStart1;
                  info!("RDS STATE_START010 Abbruch -> STATE_START1.");
                }
              } else {
                if gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                  //1 gelesen, Abbruch
                  count = 0;
                  state = StateRdsRx::StateStart1;
                  info!("RDS STATE_START010 Abbruch -> STATE_START1.");
                } else {
                  //0, OK, Startsquenz ist fertig!
                  count = 0;
                  state = StateRdsRx::StateData;
                  debug!("RDS STATE_START010 -> STATE_DATA.");
                }
              }
            }
            StateRdsRx::StateData => {
              values[count / 8] = (values[count / 8] << 1)
                | if gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                  1
                } else {
                  0
                };
              count += 1;
              if count >= bytes_erwartet {
                state = StateRdsRx::StateCheck;
                count = 0;
                debug!("RDS STATE_DATA -> STATE_CHECK.");
              }
            }
            StateRdsRx::StateCheck => {
              rdsCheckSumme = (rdsCheckSumme << 1)
                | if gpio_mfx_rds_dat.read_value().unwrap() == GpioValue::High {
                  1
                } else {
                  0
                };
              count += 1;
              if count >= 8 {
                //Checksumme vollständig eingelesen
                state = StateRdsRx::StateFinal;
                debug!("RDS STATE_CHECK -> STATE_FINAL");
              }
            }
            StateRdsRx::StateFinal => {} //Nichts mehr, gesamte Schleife wird sowieso abgebrochen
          }
        }
      }
      if state == StateRdsRx::StateFinal {
        //Noch Checksumme prüfen
        let mut checksum = 0x00FF as u16;
        for i in 0..bytes_erwartet {
          checksum ^= (checksum << 1) ^ (checksum << 2);
          checksum ^= values[i] as u16;
          if (checksum & 0x0100) != 0 {
            checksum = checksum ^ 0x0107;
          }
          if (checksum & 0x0200) != 0 {
            checksum = checksum ^ 0x020E;
          }
        }
        if checksum as u8 == rdsCheckSumme {
          result = Some(values[0..bytes_erwartet].to_vec());
          debug!("RDS Checksumme OK.");
        } else {
          warn!("RDS Checksumme falsch.");
        }
      }
      if tx.send(result).is_err() {
        warn!("MFX RDS thread tx.send() fail.");
        break;
      }
    } else {
      warn!("MFX RDS thread rx.recv() fail.");
      break;
    }
  }
}
