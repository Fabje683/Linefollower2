# Gebruiksaanwijzing

### opladen / vervangen batterijen
De line follower maakt gebruik van twee in-serie-geschakelde CGR18650CG Li-ion batterijen. Deze zitten in hun houder gemonteerd achteraan de linefollower. Deze batterijen leveren een maximum spanning van 4.2V elk, in schakeling dus 8.4V. Wanneer u merkt dat de line follower trager begint te rijden, deze plots afschakeld zal blijken dat de batterijen onder de minimale benodigde spanning gezakt zijn om een correcte of optimale werking te kunnen garanderen.

Het is de bedoeling dan de batterijen voorzichtig uit de houder van de wagen te halen (deze spant relatief hard) en de batterijen op te laten met de daartoe voorziene lader uit de BOM. Deze lader is geschikt om Li-ion cellen te laden. Op het scherm van de lader zal u kunnen aflezen welke spanning er momenteel over elke cel staat en tot waar de lader deze zal opladen. Blijkt de maximum spanning verkeerd ingesteld dan is het van gigantisch belang dat u deze bijstelt (zie vorige alinea) de lader zal deze cellen opladen volgens 0.5A, 1A of 2A, afhankelijk wat u instelt op de lader en watde voeding van de lader kan levere

### draadloze communicatie
Bij de Arduino UNO zal het nodig zijn om de buetooth module te verwijderen wanneer men wilt communiceren via de USB-poort. Na het uploaden van het programma of vanaf wanneer draadloze communicatie gewenst is is het de bedoeling dat de gebruiker de usb-kabel loskoppelt en dan pas de HC-05 opnieuw connecteert met de Arduino.
#### verbinding maken
Zoek bij instellinen -> bluetooth op het mobiele apparaat naar de HC-05 module. Connecteer en wanneer men vraagt naar een pincode geeft u '0000' of '1234' in. Download en open de app 'Serial Bluetooth terminal' ga naar devices en selecteer de HC-05. Op het beginscherm kan u voortaan bovenaan op het connectie stuk klikken om sneller verbinding te maken. Op de terminal komt 'connected' te staan wanneer alles goed verlopen is.

#### commando's
debug [on/off]  
start  
stop  
set cycle [µs]  
set power [0..255]  
set diff [0..1]  
set kp [0..]  
set ki [0..]  
set kd [0..]  
calibrate black : 
calibrate white :
run :

### kalibratie
Vooraf het starten van het autootje is het belangrijk steeds de sensor te kalibreren, andere spanning, lichtinval of inkt verschil kan lijden tot verkeerdelijk geïterpreteerde kleur en dus een volledige regelaar die steund op verkeerde waarden. Plaats de sensor op een volledig zwart geprint vlak van het circuit en geef thet commando 'calibrate black' in, wanneer de Arduino 'done' terugstuurd is deze klaar. Dan, plaats de sensor op een volledig wit stuk van het circuit en geef thet commando 'calibrate white' in, wanneer de Arduino 'done' terugstuurd is deze klaar. De sensoren zijn nu correct gekalibreert. 

### settings
De robot rijdt stabiel met volgende parameters: cycle:? power: ? diff:?kp: ? kd: ? ki: 0

### start/stop button
De button bevindt zich op het breadbord. Startprocedure met button: Plaats de wagen op het circuit, op een lijn. Wanneer de gebruiker op de knop drukt (verbonden met de interrupt pin 2) zal een drie keer knipperen (drie seconden) voordat het wagentje vertrekt. Stopprocedure met button: Wanneer men nog maals op de knop drukt (er wordt ook gekeken of de variabele 'run' op true staat) zal de variabele 'run' onmiddelijk op false gezet worden en zal het wagentje onmiddelijk stoppen. Praktisch kan het wagentje stoppen met de button op het circuit zeer uitdagend zijn, het gebruik van de button is echter wel geschikt wanneer de gebruiker de auto in de hand heeft.
