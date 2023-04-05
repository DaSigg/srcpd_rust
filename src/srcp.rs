//! srcp Protokoll Implementierung
//! - Info mode
//! - Command mode
//!   - INIT
//!   - SET
//!
//! INI File:
//! [srcp]
//! port = xxxxxx

use std::{
  collections::HashMap,
  io::{Read, Write},
  net::{TcpListener, TcpStream},
  sync::{
    mpsc::{self, Receiver, Sender},
    Mutex,
  },
  thread,
  time::{SystemTime, UNIX_EPOCH},
};

use log::{error, info, warn};

use crate::srcp_server_types::Message;

//Unterstützte SRCP version
const SRCP_VERSION: &'static str = "0.8.4";

//Verwaltung aller Sender zu allen angemeldeten SRCP Info Clients
//Key ist die Session ID
struct SenderSession {
  sender: Sender<Message>,
  session_id: u32,
}
static ALLE_SRCP_INFO_SENDER: Mutex<Vec<SenderSession>> = Mutex::new(Vec::new());

//enum für SRCP Command- oder Infomode
#[derive(Debug)]
enum SrcpMode {
  Command,
  Info,
}

/// Read line function die tolerant gegenüber nicht ASCII Zeichen ist, diese werden ignoriert.
/// Es wird jeweils bis \n gelesen. Blockiert solange kein \n gelesen wurde oder Verbindung abbricht.
/// Liefert Err bei Verbindungsabbruch
/// # Arguments
/// * client_stream - TCP Stream von dem gelesen werden soll
/// * line - Gelesene Zeile
fn read_line(mut client_stream: &TcpStream, line: &mut String) -> Result<(), ()> {
  let mut buffer: [u8; 1] = [0; 1];
  line.clear();
  loop {
    client_stream.read_exact(&mut buffer).or(Err(()))?;
    match buffer[0] {
      b'\n' => break,
      b' '..=b'~' => line.push(char::from_u32(buffer[0].into()).unwrap()),
      _ => {} //Ignorieren
    }
  }
  Ok(())
}

/// SRCP Message zum Client senden
/// Liefert Err bei Verbindungsabbruch
/// # Arguments
/// * client_stream - TCP Stream von dem gelesen werden soll
/// * msg - Die zu sendene Message. Diese wird am Anfang mit Timestamp ergänzt und am Schluss mit\n
fn send_srcp_message(mut client_stream: &TcpStream, msg: &str) -> Result<(), String> {
  let time = SystemTime::now()
    .duration_since(UNIX_EPOCH)
    .expect("Time went backwards");
  let text = time.as_secs().to_string()
    + "."
    + format!("{:0>3}", time.subsec_millis()).as_str()
    + " "
    + msg
    + "\n";
  client_stream
    .write(text.as_bytes())
    .or(Err("SRCP Write to client Error"))?;
  Ok(())
}

/// SRCP Error Message zum Client senden
/// Liefert Err bei Verbindungsabbruch
/// # Arguments
/// * client_stream - TCP Stream von dem gelesen werden soll
/// * err_code - SRCP Errorcode
/// * msg - Error Message
fn send_srcp_error(client_stream: &TcpStream, err_code: &str, msg: &str) -> Result<(), String> {
  send_srcp_message(client_stream, &format!("{} ERROR {}", err_code, msg))
}

/// SRCP Server Handshake mit Client.
/// Liefert den gewünschten SRCP Mode oder Error
/// # Arguments
/// * client_stream - TCP Stream von/zu diesem Client
/// * session_id - Die zu verwendende Session ID
fn handle_srcp_handshake(
  mut client_stream: &TcpStream, session_id: u32,
) -> Result<SrcpMode, String> {
  let mut line = String::new();
  //SRCP Willkommensmessage senden
  //srcpd Vx.x.x; SRCP x.x.x
  client_stream
    .write(
      format!(
        "srcpd V{}; SRCP {}\n",
        env!("CARGO_PKG_VERSION"),
        SRCP_VERSION
      )
      .as_bytes(),
    )
    .or(Err("SRCP Client Write fail"))?;
  loop {
    //Warten auf gewünschten Mode
    match read_line(client_stream, &mut line) {
      Err(()) => return Err(format!("SRCP read_line Error")),
      Ok(_len) => {}
    }
    let mode = match line.to_uppercase().as_str() {
      "SET CONNECTIONMODE SRCP COMMAND" => SrcpMode::Command,
      "SET CONNECTIONMODE SRCP INFO" => SrcpMode::Info,
      _ => {
        if line.starts_with("SET PROTOCOL SRCP") {
          //Wird ignoriert, eibfach mit OK beantworten
          send_srcp_message(client_stream, "201 OK PROTOCOL SRCP")?;
        } else {
          warn!("Ungültiges SRCP Kommando empfangen: {}", line);
          send_srcp_error(client_stream, "401", "unsupported connection mode")?;
        }
        continue;
      }
    };
    send_srcp_message(client_stream, "202 OK CONNECTIONMODE")?;
    //Warten auf GO
    read_line(client_stream, &mut line).or(Err("SRCP read_line Errro"))?;
    match line.to_uppercase().as_str() {
      "GO" => (),
      _ => {
        return Err(format!("SRCP GO erwartet: {}", line));
      }
    };
    //Start neue Session
    send_srcp_message(client_stream, format!("200 OK GO {}", session_id).as_str())?;
    return Ok(mode);
  }
}

/// Info Mode SRCP Client bedienen
/// # Arguments
/// * client_stream - TCP Stream von/zu diesem Client
/// * session_id - Die zu verwendende Session ID
/// * all_cmd_tx - Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
fn handle_srcp_infomode(
  mut client_stream: &TcpStream, session_id: u32, all_cmd_tx: &HashMap<usize, Sender<Message>>,
) {
  //No blocking read um Rx Buffer leeren zu können
  client_stream
    .set_nonblocking(true)
    .expect("handle_srcp_infomode set_nonblocking call failed");
  //Channel zum Empfang von Info Message aufbauen und anmelden
  let (info_tx, info_rx) = mpsc::channel();
  //und anmelden
  {
    let mut guard = ALLE_SRCP_INFO_SENDER.lock().unwrap();
    let prot_alle_info_sender = &mut *guard; // take a &mut borrow of the value
    prot_alle_info_sender.push(SenderSession {
      sender: info_tx,
      session_id: session_id,
    });
  }
  //Allen Servern den neuen Info Mode Client mitteilen so dass diese ein Update aller Zustände senden können
  let message = Message::new_info_client(session_id);
  for (_, sender) in all_cmd_tx {
    sender
      .send(message.clone())
      .expect("handle_srcp_infomode Error Send to Server fail");
  }
  //Und ab jetzt einfach alle Info Meldungen weitersenden
  loop {
    let srcp_msg = info_rx
      .recv()
      .expect("handle_srcp_infomode Error recv")
      .to_string();
    if send_srcp_message(client_stream, srcp_msg.as_str()).is_err() {
      //Abbruch, Client ist gestorben
      break;
    }
    //Für Info Verbindungen wird nichts empfangen. Zur Sicherheit Eingangsbuffer löschen
    let mut buf = vec![];
    let _ = client_stream.read_to_end(&mut buf); //Alle Fehler ignorieren
  }
  info!("SRCP Info Client beendet")
}

/// SRCP Server Thread für einen Client
/// # Arguments
/// * client_stream - TCP Stream von/zu diesem Client
/// * session_id - Die zu verwendende Session ID
/// * all_cmd_tx - Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
fn handle_srcp_connection(
  client_stream: &TcpStream, session_id: u32, all_cmd_tx: HashMap<usize, Sender<Message>>,
) {
  match handle_srcp_handshake(client_stream, session_id) {
    Err(msg) => {
      error!("SRCP Handshake Error: {}", msg);
      return;
    }
    Ok(mode) => {
      info!(
        "Neuer Client SRCP Mode={:?} session_id={}",
        mode, session_id
      );
      match mode {
        SrcpMode::Command => todo!(),
        SrcpMode::Info => handle_srcp_infomode(client_stream, session_id, &all_cmd_tx),
      }
    }
  }
}

/// SRCP Server der auf eingehende Verbindungen wartet, diese entgegennimmt und für jede Verbindung
/// einen Rx und Tx Thread startet
/// # Arguments
/// * port - TCP Port auf dem der Server gestartet werden soll
/// * watchdog - true: Watchdog aktiv -> automatische Ausschalting Power wenn zu lange kein neuer Befehl empfangen wird.
/// * all_cmd_tx - Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
fn srcp_server(port: u16, _watchdog: bool, all_cmd_tx: &HashMap<usize, Sender<Message>>) -> ! {
  //TODO watchdog
  let server_adr = format!("0.0.0.0:{}", port);
  info!("Start SRCP Server: {}", server_adr);
  let listener = TcpListener::bind(server_adr).expect(
    format!(
      "SRCP Server konnte nicht auf Port {} gestartet werden",
      port
    )
    .as_str(),
  );
  let mut session_id: u32 = 0;
  loop {
    info!("Warte auf SRCP Server Client");
    let (client_stream, addr) = listener.accept().expect("SRCP Server Accept fail");
    info!("SRCP Server neuer Client:{}", addr);
    session_id = session_id + 1;
    //Alle Sender müssen geklont werden damit sie im anderen Thread verwendet werden können
    let all_cmd_tx_kopie = all_cmd_tx.clone();
    //Neuer Thread für diesen Client starten
    thread::spawn(move || {
      handle_srcp_connection(&client_stream, session_id, all_cmd_tx_kopie);
    });
  }
}

/// Senden einer SRCP Info Massage an Clients
/// Wenn eine Message nicht versendet werden konnte, dann wird der entsprechende Client gelöscht.
/// # Arguments
/// * msg - Die zu versendende Message
/// * alle - Wenn true, dann wird die Message an alle aktuell engemeldeten Info Clients versendet
/// * session_id - Wenn all == false, dann wird die Message nur an Client mit dieser Session ID versendet
fn send_info_msg(msg: &Message, alle: bool, session_id: u32) {
  let mut guard = ALLE_SRCP_INFO_SENDER.lock().unwrap();
  let prot_alle_info_sender = &mut *guard; // take a &mut borrow of the value
  let mut i = 0;
  while i < prot_alle_info_sender.len() {
    if alle || (prot_alle_info_sender[i].session_id == session_id) {
      if prot_alle_info_sender[i].sender.send(msg.clone()).is_err() {
        //Diesen Client gibt es nicht mehr
        info!(
          "dispachter_srcp_info delete Client session_id={}",
          prot_alle_info_sender[i].session_id
        );
        prot_alle_info_sender.remove(i);
      } else {
        i += 1;
      }
    } else {
      i += 1;
    }
  }
}

/// Dispatcher für alle SRCP Info Messages von allen Servern zu Weiterleitung an alle
/// aktuell angemeldeten Info Clients
/// # Arguments
/// * info_rx - Channel über die die Info Messages empfangen werden
fn dispachter_srcp_info(info_rx: Receiver<Message>) {
  loop {
    let msg = info_rx
      .recv()
      .expect("Error: dispachter_srcp_info info_rx.recv() fail");
    if let Message::SRCPMessage { ref srcp_message } = msg {
      match srcp_message.srcp_message_dir {
        crate::srcp_server_types::SRCPMessageDir::Info => {
          //Muss an alle angemeldeten SRCP Info Clients gesendet werden
          send_info_msg(&msg, true, 0);
        }
        crate::srcp_server_types::SRCPMessageDir::InfoSession { session_id } => {
          //Nur an Client mit session_id
          send_info_msg(&msg, false, session_id);
        }
        _ => {} //Nur Info Messages sind relevant.
      }
    }
  }
}

/// Startet den srcp Server
/// # Arguments
/// * config_file_values - Gesamtes Konfigfile
/// * all_info_rx - Alle Channel Receiver der Info Messages aller Server
/// * all_cmd_tx - Alle Channel Sender für Kommandos zu den SRCP Servern. Key ist die Busnummer.
pub fn startup(
  config_file_values: &HashMap<String, HashMap<String, Option<String>>>,
  info_rx: Receiver<Message>, all_cmd_tx: &HashMap<usize, Sender<Message>>,
) -> Result<(), String> {
  let port = config_file_values
    .get("srcp")
    .ok_or("Keine [srcp] Abschnitt in Konfiguration")?
    .get("port")
    .ok_or("Keine [srcp] port-Angabe in Konfigfile")?
    .as_ref()
    .ok_or("[srcp] port-Angabe ohne Wert")?
    .parse::<u16>()
    .ok()
    .ok_or("[srcp] port muss eine Zahl sein")?;
  //Wenn wir bis hierhingekommen sind, dann gibt es den [srcp] Abschnitt auf jeden Fall
  let watchdog = config_file_values
    .get("srcp")
    .expect("Keine [srcp] Abschnitt in Konfiguration")
    .get("watchdog")
    .is_some();
  info!("srcp start port={port} watchdog={watchdog}");

  //Info Message Dispacther Thread starten
  //Alle Infos Messages der verschiedenen srcp_server_ Instanzen werden von diesem Thread an alle angemeldeten
  //Clients mit Info Mode gesendet
  thread::spawn(move || {
    dispachter_srcp_info(info_rx);
  });

  //Hier geht es weiter mit als Hauptthread der auf eingehende Verbindungen wartet
  //und die Verbindung zwischen den für die Verbindungen gestarteten SRCP Servern und den Bus-Servern herstellt
  srcp_server(port, watchdog, all_cmd_tx);
}
