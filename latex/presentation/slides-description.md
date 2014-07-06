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

### 4. Slajd - Grafički prikaz projekta
* Ova slika grafički prikazuje zadatak diplomskog rada. 
* Rad se može podjeliti u dva dijela *snimanje* i *izgradanju*.
* U prvom djelu je bilo potrebno upotrebom Kinect kamere i RGBDSlam
  programa snimiti scenu i objekte i dobiti 3D oblak točaka.
* Točnije, svaka snimka je jedan oblak točaka i taj oblak točaka je dio
  globalnog oblaka točaka sastavljenog od više snimki uzetih iz različitih
  pogleda odnosno položaja kamere.
* Zatim koristeći PCL biblioteku razviti program kojeg sam nazvao
  mesh-reconstruction koji će od 3D oblaka točaka izgraditi 3D model
  pomoću mreže trokuta.
* Potrebno je ispitati funkcionalnosti i kvaltetu opisanog postupka
  kao i kvalitetu dobivenog 3D modela izgradnjom nekoliko 3D modela
  objekta i scena.

### 5. Slajd - Sadržaj
* Gotovi smo s uvodom sada slijedi poglavlje pregled upotrebljenih
  tehnologija i algoritama u kojem ću **ukratko** objasniti nabrojane
  tehnologije.

### 6. Slajd - Kinect 3D kamera
* Kineckt 3D kamera sastoji se od 3D dubinskog senzora i RGB kamere.
* Ona daje sliku u boji sinkroniziranu s dubinskom slikom.
* Radi se o VGA rezoluciji i osvježavanju od 30 Hz.
* Dubinski senzor sastoji se od IR projektora i IR kamere te ima
  ograničenje u dometu na 0.8m - 3.5m.

### 7. Slajd - Kinect 3D kmaera - princip rada dubinskog senzora
* Zasniva se na principu strukturirane svjetlosti.
* IR projektor projicira jedinstven uzorak točkastih mrlja.
* IR kamera hvata reflektirane IR mrlje te se računanje
  dubine odvija na kameri stereo triangulacijom.

### 8. Slajd - ROS - Operacijski sustav za robote
* ROS - Robot operating system je meta operacijski sustav za robote
  zamišljen da olakšava razvijanje aplikacija za robote.
* ROS pruža hardversku apstrakciju, upravljačke programe, biblioteke,
  alate, komunikaciju, pakete...
* RGBDSlam program korišten za snimanje scene je baziran na ROSu

### 9. Slajd - PointCloud biblioteka funkcija i algoritama 
* Sljdeće potpoglavlje je o PCL biblioteci, PCL je biblioteka funkcija i
  algoritama za rad s oblakom točaka.
* Oblak točaka je 3D skup točka dobiven spajanje slike u boji s
  dubinskom slikom
* PCL biblioteka je također dio ROSa
* te je upotrebljena za razvoj mesh-reconstruction programa.

### 10. Slajd - Metoda istovremene lokalizacije i mapiranja - SLAM
* Metoda istovremene lokalizacije i mapiranja se bavi rješavanjem
  problema izgradnje karte nepoznate okoline i istovremene navigacije
  upotrebom te karte.
* Cilj procesa je korištenje percepcije okoline za pozicioniranje
  robota/kamere.
* To se postiže detektiranjem orjentira/značajki. 
* Jezgra procesa je EKF - on je odgovoran za ažuriranje pozicije na
  kojoj robot misli da se nalazi 

### 11. Slajd - Poisson algoritam za rekonstrukciju površine 
* Pristupa problemu rekonstrukcije površine rješavanjem Poissonove
  jednadžbe.
* Koja kaže da je divergencija orjentiranih normala s površine modela
  jednaka divergenciji gradijenta indikacijske funkcije chi.
* Točnije gradijent indikacijske funkcije je polje vektora koje je
  uglavnom popunjeno nulama osim na mjestima blizu površine modela gdje
  je jednako unutrašnjim normalama površine

### 12. Slajd - Sadržaj
* Gotovo je poglavlje o korištenim tehnologijama i algoritama i slijedi
  poglavlje o snimanju i izgradnji 3D modela scene.

### 13. Slajd - Opis rada RGBDSlam programa
* RGBDSlam program procesira snimke u četiri koraka:
* Prvo se dohvaćaju vizualne značajke iz ulaznih slika u boji i
  spraviaju s prethodno pronađenim značajkama - SURF, SIFT
* Zatim se vrši evalucija dubinskih slika na lokacijama pronađenih
  značajki čime se ostvaruju 3D korespodencije između slika
* Tada se upotrebom tih korespodencija estimira položaj kamere RANSAC
  algoritmom.

### 14. Slajd - Opis rada RGBDSlam programa
* Kako estimirani položaju kamera nisu globalno konzistentni vrši se
  optimiziranje grafa položaja g2o 
* I tada program daje globalno konzistentan oblak točaka u boji.

### 15. Slajd - Prikaz RGBDSlam programa
* Na ovom slajdu vidimo prikaz RGBDSlam programa koji se sastoji od
  četiri prozora. 
* Donji lijevi prozor prikazuje ulaznu sliku s rgb kamere.
* Srednji prozor prikazuje dubinsku sliku.
* Donji desni prikazuje pronađene značajke nad ulaznom slikom.
* Gornji prozor je rezerviran za prikaz sastavljenog oblaka točaka.

### 16. Slajd - Pregled razvijenog programa mesh-reconstruction
* Izgradnja 3D modela scene pomoću mreže trokuta se odvija pomoću
  programa mesh-reconstruction
* Program je podijeljen u pet logičkih cijelina. 
* Prvo se odabire oblak točaka nad kojim se žele izvršiti akcije.
* Zatim se redom izvršavaju akcije: reduciranje oblaka točaka, 
  odbacivanje odudarajućih vrijednosti, izrađivanje mreže trokuta i
  prikazivanje mreže trokuta.

### 17. Slajd - Grafičko sučelje programa mesh-reconstruction
* Osim komandno linijske verzije mesh-reconstruction programa razvijena
  je i verzija s grafičkim sučeljem.
* Program je napravljen korištenjem Qt frameworka za razvoj više
  platformskih aplikacija.
* Sučelje je jednostavno, prvo se odabire datoteka s oblakom točaka ili
  mrežom trokuta i onda se izvršavaju akcije nad njima.
* Razlika u odnosnu na komandno linijsku verziju je u mogućnosti
  podešavanja parametara Poisson algoritma.

### 18. Slajd - mesh-reconstruction GUI - prikaz izrađene mreže
* Osim glavnog prozora za izradu mreže postoji i prozor za vizualizaciju
  u kojem je moguće pregledati izrađeni model.
* Moguće je mjenjati prezentaciju modela, trenutno je prikazan model
  površinskom prezentacijom, a moguće je i odabrat wireframe
  representation sto omogućava prikaz mrežom trokutića

### 19. Slajd - sadržaj
 
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

