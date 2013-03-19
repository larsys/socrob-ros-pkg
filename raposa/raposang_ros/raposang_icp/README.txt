1º Correr a topicos no raposa que dao a odometria

2º roslaunch raposang_icp worldmap2.launch
Nota: cria a pasta onde vai escrever os mapas 

3º roslaunch raposang_icp demo_odometry.launch
Nota: usa a odometria do robo, ou as features, e o icp
Nota2: é importante a odometria ja estar a ser fornecida nos topicos

4º Correr o driver do miguel: roslaunch kinect_driver kinect_driver.launch
Nota: eu estou a ir buscar os point clouds ao "/kinect/depth_camera/pointcloud_color_decompress", 
se voces nao usarem o compressor do miguel têm que alterar o nome do topico no demo_odometry.launch, ou entao usar remaps


No demo_odometry.launch tem uma variavel que se chama "useicp" que pode ser true ou false, conforme voces queiram usar o icp
ou fazer os mapas apenas com odometria do robo.



Para correr offline:
1º abrir 4 terminais

2º 
   terminal1 correr:  roslaunch kinect_driver kinect_driver_decom.launch 
   terminal2 correr:  roslaunch raposang_icp worldmap2.launch
   terminal3 correr:  roslaunch raposang_icp demo_odometry.launch
   terminal4 correr:  rosbag play --pause bag_pedro.bag  (carreguem na barra de espaços quando quiserem iniciar)
   
   


