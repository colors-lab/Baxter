
Synaptic'ten resimdeki paketleri indirdikten sonra,

https://www.20papercups.net/programming/kinect-on-ubuntu-with-openni/
bu  sitedeki adım 3'ü yapıyoruz.

Daha sonra,
https://github.com/arnaud-ramey/NITE-Bin-Dev-Linux-v1.5.2.23
buradan NITE 1.5.2'yi kuruyoruz.

http://wiki.ros.org/openni_tracker
Openni tracker paketini de kurduktan sonra, kullanıma hazır hale geliyor.

Kullanmak için 4 terminalde . baxter.sh yazdıktan sonra

roslaunch openni_launch openni.launch
rosrun listener listener1
rosrun listener tuluhan
rosrun openni_tracker openni_tracker

kodlarını çalıştırıyoruz.


