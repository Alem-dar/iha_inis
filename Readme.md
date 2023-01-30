# iha_inis


Proje hakkında video linki --> https://drive.google.com/drive/folders/1ASdnBdnY_AWT82pAWxVbZfxohb6oN-rd?usp=sharing

Proje Ortamı Çalıştırılması ve Komutlar
İlk olarak Gazebo ortamının başlatılması gerekir. Bunu yapabilmek için hector_quadrotor_demo/launch klasörü içine outdoor.launch dosyası oluşturulmalı. Daha sonra terminalden

“cd catkin_ws/devel” yazılıp o konuma gidilir. Burada “source setup.bash” komutu yazılır sonra Gazebo ortamı başlatılır. Bu başlatma için “ roslaunch hector_quadrotor_demo outdoor.launch” komutu çalıştırılır ve Gazebo ortamı başlar.

Daha sonra terminalde başka bir pencerede Hector Quadrotor’un motorlarının çalıştırılması gerekir. Bunun için;
rosservice call /enable_motors “enable: true” komutu yazılır. Enable : true komutu geri dönerse Hector uçuşa hazır hale gelir.

Artık yazdığımız kodların çalıştırılması gerekir. Bunun için terminalden sırayla pioneer3at_konum.py, markerla_inis_dene.py ve arayuz.py çalıştırılır. 
Bu sayede proje başarılı bir şekilde başlatılmış olur.

Oluşturulan veya Değişiklik Yapılan Dosyalar
İlk olarak AR-Tag oluşturuldu. Bunun için terminale “rosrun ar_track_alvar createMarker”
komutu yazıldı. Daha sonra istenilen bilgiye “1234” yazıldı. Sonra 2 kez Enter’a basıldı. Tekrar bilgi istediğinde “-1” yazıldı ve MarkerData_1234.png oluştu.

Bu oluşumdan sonra Blender adlı uygula yardımıyla oluşan fotoğraf 3 boyutlu simülasyon için bir küpe basıldı. Daha sonra ar_2020.dae ve ar_2020.urdf adlı dosyalar oluşturuldu. Burada MarkerData_1234.png ve ar_2020.dae dosyalarının konumu hector_quadrotor_description/meshes klasörü içerisinde olmalıdır. Oluşturulan diğer dosya yani ar_2020.urdf ise hector_quadrotor_description/urdf klasörü içerisinde bulunmalıdır.

Daha sonra oluşturulan bu AR-Tag, Pioneer-3AT üzerine eklemek için pioneer3at_body.xacro dosyasına eklendi. Gene aynı dosyaya AR-Tag’i üzerine oturtabilmek için platform ve bunun yanında GPS bilgisi alabilmek için GPS eklendi.







