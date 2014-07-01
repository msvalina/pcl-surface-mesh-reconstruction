## Opis slajdova prezentacije

### 1. Slajd
* Dobar dan svima! Moje ime je Marijan Svalina i sada ću pričati o svome
  diplomskom radu.
* Prije početka izlaganja htio bih pozdraviti uvažene profesore članove
  povjerenstva za obranu...
* Također želim pozdraviti obitelj i prijatelje, hvala vam što ste našli
  vremena i uveličali ovaj trenutak.
* Naslov mog diplomskog rada je **Izgradnja 3D modela scene pomoću 3D
  kamere** pa krenimo s prezentacijom.

### 2. Slajd
* Slijedi pregled prezentacije, prezentacija je podijeljna u pet poglava:
* Uvod u kojem su prikazani temelji rada i dan je zadatak DR
* zatim pregled upotrebljenih tehnologija i algoritama
* pa snimanje i izgradanja 3D modela scene
* te rezultati pokusa i zaključak.
**Pa idemo redom.** 

### 3. Slajd
* Dostupnost jeftinog 3D senzora, Kineck kamere 2010. godine uvelike
  pridonosi razvoju računalnog vida. 
* Kamera je došla na tržište kao dio Microsoft Xbox igračke konzole.
* Frekvencijom od 30Hz daje sliku u boji sinkroniziranu s **dubinskom**
  slikom.
* Time omogućava na prirodniji način riješavanje problema računalnog
  vida. 
* To se i dogodilo te su znanstvenici, programeri i hakeri razvili
  upravljačke programe, alate i algoritme za korištenje Kinecta.
* Većina tog softvera je objavljena pod slobodnim licencama, te su ti
  programi temelj ovog diplomskog rada.

### 4. Slajd
* Ova slika grafički prikazuje zadatak diplomskog rada.
* Upotrebom Kinect kamere i RGBDSlam programa potrebno je bilo snimiti
  scenu i objekte (zec) i dobiti 3D oblak točaka.
* Zatim koristeći PCL biblioteku razviti program kojeg sam nazvao
  mesh-reconstruction koji će od 3D oblaka točaka izgraditi 3D model
  pomoću mreže trokuta.
* Potrebno je ispitati funkcionalnosti i kvaltetu opisanog postupka
  kao i kvalitetu dobivenog 3D modela izgradnjom nekoliko 3D modela
  objekta i scena.

### 5. Slajd
* Gotovi smo s uvodom sada slijedi poglavlje pregled upotrebljenih
  tehnologija i algoritama u kojem ću **ukratko** objasniti nabrojane
  tehnologije.

### 6. Slajd
* Kineckt 3D kamera sastoji se od 3D dubinskog senzora i RGB kamere.
* Ona daje sliku u boji sinkroniziranu s dubinskom slikom.
* Radi se o VGA rezoluciji i osvježavanju od 30 Hz.
* Dubinski senzor sastoji se od IR projektora i IR kamere te ima
  ograničenje u dometu na 0.8m - 3.5m.

### 7. Slajd
* Zasniva se na principu strukturirane svjetlosti.
* IR projektor projicira jedinstven uzorak točkastih mrlja.
* IR kamera hvata reflektirane IR mrlje te se računanje
  dubine odvija na kameri stereo triangulacijom.

### 8. Slajd
* ROS - Robot operating system je meta operacijski sustav za robote
  zamišljen da olakšava razvijanje aplikacija za robote.
* ROS pruža hardversku apstrakciju, upravljačke programe, biblioteke,
  alate, komunikaciju, pakete...
* RGBDSlam program korišten za snimanje scene je baziran na ROSu

### 9. Slajd
* Sljdeće potpoglavlje je o PCL biblioteci, PCL je biblioteka funkcija i
  algoritama za rad s oblakom točaka.
* Oblak točaka je 3D skup točka dobiven spajanje slike u boji s
  dubinskom slikom
* PCL biblioteka je također dio ROSa
* te je upotrebljena za razvoj mesh-reconstruction programa.

### 10. Slajd
* Metoda istovremene lokalizacije i mapiranja se bavi rješavanjem
  problema izgradnje karte nepoznate okoline i istovremene navigacije
  upotrebom te karte.
* Cilj procesa je korištenje percepcije okoline za pozicioniranje
  robota/kamere.
* To se postiže detektiranjem orjentira/značajki. 
* Jezgra procesa je EKF - on je odgovoran za ažuriranje pozicije na
  kojoj robot misli da se nalazi 

### 11. Slajd
* Pristupa problemu rekonstrukcije površine rješavanjem Poissonove
  jednadžbe
* Jednadžba opisuje odnosno algoritam se oslanja na ideju da postoji
  veza između orjentiranih normala uzetih s površine modela i
  indikacijske funkcije modela.
* Točnije gradijent indikacijske funkcije je polje vektora koje je
  uglavnom popunjeno nulama osim na mjestima blizu površine modela gdje
  je jednako unutrašnjim normalama površine

### 12. Slajd
* Gotovo je poglavlje o korištenim tehnologijama i algoritama i slijedi
  poglavlje o snimanju i izgradnji 3D modela scene.

### 13. Slajd
* 

