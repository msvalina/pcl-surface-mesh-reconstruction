## Opis slajdova prezentacije

### 1. Slajd - Naslov
* Dobar dan svima! Moje ime je Marijan Svalina i sada ću pričati o svome
  diplomskom radu.
* Prije početka izlaganja htio bih pozdraviti uvažene profesore članove
  povjerenstva za obranu...
* Također želim pozdraviti obitelj i prijatelje, hvala vam što ste našli
  vremena i uveličali ovaj trenutak.
* Naslov mog diplomskog rada je **Izgradnja 3D modela scene pomoću 3D
  kamere** pa krenimo s prezentacijom.

### 2. Slajd - Pregled prezentacije
* Slijedi pregled prezentacije, prezentacija je podijeljna u pet poglava:
* Uvod u kojem je opisan zadatak DR
* zatim pregled upotrebljenih tehnologija i algoritama
* pa snimanje i izgradanja 3D modela scene
* te rezultati pokusa i zaključak.
**Pa idemo redom.** 

### 4. Slajd - Grafički prikaz projekta - snimanje
* Ova slika grafički prikazuje zadatak diplomskog rada. 
* Ukratko zadatak je bio *snimti* scenu, *izgradati* model i
  *ispitati kvalitetu*.
* Konkretnije trebalo je upotrebom Kinect kamere i RGBDSlam
  programa snimiti scenu i objekte i dobiti 3D oblak točaka.
* Točnije, svaka snimka je jedan oblak točaka i taj oblak točaka je dio
  globalnog oblaka točaka sastavljenog od više snimki uzetih iz različitih
  pogleda odnosno položaja kamere.

### 5. Slajd - Grafički prikaz projekta - izgradnja
* Zatim je trebalo dobiveni oblak točaka iskoristiti za izgradnju 3D
  modela scene pomoću mreže trokuta.
* Odnosno razviti program upotrebom PCL biblioteke izgraditi 3D model iz
  snimljenog oblaka točaka.

### 6. Slajd - Grafički prikaz projekta - ispitivanje
* Potrebno je ispitati funkcionalnosti i kvaltetu opisanog postupka
  kao i kvalitetu dobivenog 3D modela izgradnjom nekoliko 3D modela
  objekta i scena.

### 7. Slajd - Sadržaj
* Gotovi smo s uvodom sada slijedi poglavlje pregled upotrebljenih
  tehnologija i algoritama u kojem ću **ukratko** objasniti najvažnije
  tehnologije.

### 8. Slajd - Kinect 3D kamera
* Kineckt 3D kamera sastoji se od 3D dubinskog senzora i RGB kamere.
* Ona daje sliku u boji sinkroniziranu s dubinskom slikom.
* Radi se o VGA rezoluciji i osvježavanju od 30 Hz.
* Dubinski senzor sastoji se od IR projektora i IR kamere te ima
  ograničenje u dometu na 0.8m - 3.5m.

### 9. Slajd - Kinect 3D kmaera - princip rada dubinskog senzora
* Zasniva se na principu strukturirane svjetlosti.
* IR projektor projicira jedinstven uzorak točkastih mrlja.
* IR kamera hvata reflektirane IR mrlje te se računanje
  dubine odvija na kameri stereo triangulacijom.

### 10. Slajd - ROS - Operacijski sustav za robote
* ROS - Robot operating system je meta operacijski sustav za robote
  zamišljen da olakšava razvijanje aplikacija za robote.
* ROS pruža hardversku apstrakciju, upravljačke programe, biblioteke,
  alate, komunikaciju, pakete...
* RGBDSlam program korišten za snimanje scene je baziran na ROSu

### 11. Slajd - PointCloud biblioteka funkcija i algoritama 
* Sljdeće potpoglavlje je o PCL biblioteci, PCL je biblioteka funkcija i
  algoritama za rad s oblakom točaka.
* Oblak točaka je 3D skup točka dobiven spajanje slike u boji s
  dubinskom slikom
* PCL biblioteka je također dio ROSa
* te je upotrebljena za razvoj mesh-reconstruction programa.

### 12. Slajd - Metoda istovremene lokalizacije i mapiranja - SLAM
* Metoda istovremene lokalizacije i mapiranja se bavi rješavanjem
  problema izgradnje karte nepoznate okoline i istovremene navigacije
  upotrebom te karte.
* Cilj procesa je korištenje percepcije okoline za pozicioniranje
  robota/kamere.
* To se postiže detektiranjem orjentira/značajki. 
* Jezgra procesa je EKF - on je odgovoran za ažuriranje pozicije na
  kojoj robot misli da se nalazi 

### 13. Slajd - Poisson algoritam za rekonstrukciju površine 
* Pristupa problemu rekonstrukcije površine rješavanjem Poissonove
  jednadžbe.
* Koja kaže da je divergencija orjentiranih normala s površine modela
  jednaka divergenciji gradijenta indikacijske funkcije chi.
* Točnije gradijent indikacijske funkcije je polje vektora koje je
  uglavnom popunjeno nulama osim na mjestima blizu površine modela gdje
  je jednako unutrašnjim normalama površine

### 14. Slajd - Sadržaj
* Gotovo je poglavlje o korištenim tehnologijama i algoritama i slijedi
  poglavlje o snimanju i izgradnji 3D modela scene.

### 15. Slajd - Opis rada RGBDSlam programa
* RGBDSlam program procesira snimke u četiri koraka:
* Prvo se dohvaćaju značajke iz ulaznih slika u boji i
  spraviaju s prethodno pronađenim značajkama - SURF, SIFT
* Zatim se radi evalucija dubinskih slika na lokacijama pronađenih
  značajki čime se ostvaruju 3D korespodencije između slika
* Tada se upotrebom tih korespodencija estimira položaj kamere RANSAC
  algoritmom.

### 16. Slajd - Opis rada RGBDSlam programa
* Kako estimirani položaju kamera nisu globalno konzistentni ako program
  može izvršiti zatvaranje petlje onda vrši optimiziranje grafa položaja 
* I tada program daje globalno konzistentan oblak točaka u boji.

### 17. Slajd - Prikaz RGBDSlam programa
* Na ovom slajdu vidimo prikaz RGBDSlam programa koji se sastoji od
  četiri prozora. 
* Donji lijevi prozor prikazuje ulaznu sliku s rgb kamere.
* Srednji prozor prikazuje dubinsku sliku.
* Donji desni prikazuje pronađene značajke nad ulaznom slikom.
* Gornji prozor je rezerviran za prikaz sastavljenog oblaka točaka.

### 18. Slajd - Pregled razvijenog programa mesh-reconstruction
* Izgradnja 3D modela scene pomoću mreže trokuta se odvija pomoću
  programa mesh-reconstruction
* Program je podijeljen u pet logičkih cijelina. 
* Prvo se odabire oblak točaka nad kojim se žele izvršiti akcije.
* Zatim se redom izvršavaju akcije: reduciranje oblaka točaka, 
  odbacivanje odudarajućih vrijednosti, izrađivanje mreže trokuta i
  prikazivanje mreže trokuta.

### 19. Slajd - Grafičko sučelje programa mesh-reconstruction
* Osim komandno linijske verzije mesh-reconstruction programa razvijena
  je i verzija s grafičkim sučeljem.
* Program je napravljen korištenjem Qt frameworka za razvoj više
  platformskih aplikacija.
* Sučelje je jednostavno, prvo se odabire datoteka s oblakom točaka ili
  mrežom trokuta i onda se izvršavaju akcije nad njima.
* Razlika u odnosnu na komandno linijsku verziju je u mogućnosti
  podešavanja parametara Poisson algoritma.

### 20. Slajd - mesh-reconstruction GUI - prikaz izrađene mreže
* Osim glavnog prozora za izradu mreže postoji i prozor za vizualizaciju
  u kojem je moguće pregledati izrađeni model.
* Moguće je mjenjati prezentaciju modela, trenutno je prikazan model
  površinskom prezentacijom, a moguće je i odabrat wireframe
  representation sto omogućava prikaz mrežom trokutića

### 21. Slajd - sadržaj
* Sada slijedi prikaz snimljenih i izgrađenih modela scena 

### 22. Slajd - Prikaz snimljenih scena
* Tijekom diplomskog rada snimljeno i izrađeno je šest scena koje su
  prikazane na ovoj slici.
* Od toga su tri snimljene ovdje na fakultetu i sad ću prikzati snimku
  hodnika, na kojoj cu i objasniti ograničenja tehnologija 
* Sam postupak daje funkcionalne rezultate ako se uzmu u obzir
  ograničenja tehnologija
* Na snimci vidimo da na prozoru
* nema točka i to je zbog ograničenja dubinskog senzora na
  kinect kameri jer senzora ima problema s staklenim i reflektirajucim
  površinama
* Na podu postoji velika rupa bez točaka, to je zato što program nije
  uspio pronaći značajke na snimci poda koje je mogao uspješno spariti
  prethodno pronađenim značajkama
* Isto tako se na oglasnoj ploči može vidjeti da program nije uspio
  optimalno zatvoriti petlju i ispraviti estimacije položaja kamere

* Što se tiče izgrdnje 3 modela pomoću mreže trokutu važno je spomenuti
  dva ograničenja:
* Algoritam izrađuje mrežu trokuta i na mjestima gdje nema točaka, što
  je kod malih rupa poželjno, a kod velikih nepoželjno
* Također program ne zadržava informaciju o boji prilikom izgradnje 
 
