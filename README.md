SRCP (https://de.wikipedia.org/wiki/Simple_Railroad_Command_Protocol) Implementierung eines DDL Servers (https://de.wikipedia.org/wiki/Digital_Direct_for_Linux) in RUST.
Einschränkungen:

    Auf SRCP Seite is nur das implementiert, was ich brauche, siehe Doku.
    Es wird nur DDL mit Ausgabe über SPI (z.B. Raspberry PI) unterstützt

Was es kann:

    DCC Servicemode, Lesen (Programmiergleis) & Schreiben (Prog. und Hauptgleis) CV’s. Hauptgleisprogrammierung Zubehördekoder.
    MM Protokolle, DCC, MFX.
    MFX Lesen und Schreiben Lokparameter, automatische Lokanmeldung mit auslesen Lokname und Funktionen.
    Servicemode für MFX.
    Es ist schnell! Optimierung der Protokollausgabe, wenn z.B. ein Protokoll eine Pause verlangt (z.B. DCC zwischen zwei Paketen an die selbe Adresse 5ms, MM5 zwischen den Paketen für 28 FS 50ms), dann wird nicht einfach gewartet sondern andere Pakete eingeschoben. Es wir keine Millisekunde verschenkt. Das ist gegenüber dem Orginal srcpd spürbar (zumindest bei meiner Anlage mit ca. 80 Loks).
    S88 Bus (auch nur über SPI).
    Konfigurierbare Oszi Triggermöglichkeit bei S88 Veränderungen, Ausgabe GA, GL, SM Kommandos.
