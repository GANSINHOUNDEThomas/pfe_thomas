# d'abord télécharger les fichiers suivant à la racine du projet sur votre machine
cd

curl -O https://raw.githubusercontent.com/GANSINHOUNDEThomas/pfe_thomas/pfe_thomas/get-docker.sh \
curl -O https://raw.githubusercontent.com/GANSINHOUNDEThomas/pfe_thomas/pfe_thomas/cohoma_humble_setup.sh \
curl -O https://raw.githubusercontent.com/GANSINHOUNDEThomas/pfe_thomas/pfe_thomas/cohoma_run.sh

chmod +x cohoma_humble_setup.sh get-docker.sh 




# si vous n'aviez pas encore le docker engine:
yes | ./get_docker.sh 

# si vous l'aviez déja:
yes | ./cohoma_run.sh 


# dans un premier temps, télécharger et copier le script à la racine 

docker run -it --privileged \
  --env=LOCAL_USER_ID="$(id -u)" \
  -v ~/ws/ws1:/root/ws:rw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -e DISPLAY=:0 \
  --network host \
  --name=cohoma_humble ubuntu:22.04 bash
  
# ensuite copiez le script de setup à la racine du docker depuis votre machine

# sur la machine:
cd 
sudo chmod a+w ws/ -R 

cp cohoma_humble_setup.sh ws/ws1  




# revenez dans le docker 
cd \
cd ws/ \
cp -r cohoma_humble_setup.sh ~/ \
cd \
yes | ./cohoma_humble_setup.sh 




# une fois les étapes précédentes terminées
# dans un second terminal : docker exec -it cohoma_humble bash
cd
cd PX4-Autopilot/ \
make px4_sitl gazebo-classic -j$(nproc) 

# stopper le simulateur avec Ctrl +C 

cd \
cd ws/ \
source /opt/ros/humble/setup.bash \
colcon build \
source install/setup.bash 
