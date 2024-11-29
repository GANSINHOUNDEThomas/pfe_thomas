# d'abord télécharger les fichiers suivant à la racine du projet sur votre machine
cd

curl -O https://raw.githubusercontent.com/GANSINHOUNDEThomas/pfe_thomas/pfe_thomas/get-docker.sh \
curl -O https://raw.githubusercontent.com/GANSINHOUNDEThomas/pfe_thomas/pfe_thomas/cohoma_humble_setup.sh \
curl -O https://raw.githubusercontent.com/GANSINHOUNDEThomas/pfe_thomas/pfe_thomas/cohoma_run.sh 

curl -O https://raw.githubusercontent.com/GANSINHOUNDEThomas/pfe_thomas/pfe_thomas/x500.sdf

chmod +x cohoma_humble_setup.sh get-docker.sh cohoma_run.sh 


# si vous n'aviez pas encore le docker engine:
yes | ./get_docker.sh 

#autoriser docker à utiliser la machine( à faire une seule fois, pas nécessaire si vous reprenez le tuto) :
echo "xhost +" >> ~/.bashrc \
source ~/.bashrc

# si vous l'aviez déja faites, cette commande lance un docker avec une image ubunut 22.04:
./cohoma_run.sh 

# ensuite ouvrez un autre terminal sur la machine:
cd \
sudo chmod a+w ws/ -R 

cp cohoma_humble_setup.sh ws/ws1  



# revenez dans le docker 
cd \
cd ws/ 
cp -r cohoma_humble_setup.sh ~/ 
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


# ENFIN vous pouvez tester la simulation(toujours DANS LE DOCKER)::
d'abord fermez le docker avec une commande exit dans le terminal principal (premier terminal lancé lors de la connexion au docker
## CONNECTER LA MANETTE PS4 par bluetooth avant de lancer le docker (IMPORTANT)
## Relancer à nouveau le docker
docker start -i cohoma_humble 

cd \
cd PX4-Autopilot/ \
make px4_sitl gazebo-classic_x500

## dans un second terminal :
docker exec -it cohoma_humble bash \
cd
MicroXRCEAgent udp4 -p 8888 

## dans un 3 ième terminal (lancer le script de la manette ps4) :
docker exec -it cohoma_humble bash \
cd \
cd ws/ 
source install/setup.bash \
ros2 run joy joy_node 

## dans un 4 ième terminal :
docker exec -it cohoma_humble bash \
cd \
cd ws/ 
source install/setup.bash \
ros2 run example_mode_with_executor_cpp example_mode_with_executor 

### cette commande lance le mode executeur que j'ai codé , il s'agit d'une machine à état (voir state_machine.ndr) que j'ai codé. (je mettrai à jour la state machine plus tard)

## MODE D'UTILISATION
ARM: appuyer sur carré \
DECOLLER : appuyez sur carré et X simultanément \
TRIANGLE : mode manuel (My_Manual_Mode) (il est accro pour le moment) il s'agit d'un raccourci mais vous pouvez aussi \
L1 (left ) ou R1 (right ) pour naviger dans la liste des différents états de navigation 
appuyer sur X pour confirmer. 
Pour l'instant, évitez de choisir le mode "Manual" 

Le mode RTL permet de retourner à la position de décollage
Le mode Hold pour le maintenir en l'air



ROND : pour atterir, le véhicule de DISARM tout seul 

Pour l'instant je n'arrive pas à utiliser les modes manuels par défaut (Position stabilized ou accro)
mais j'y travaille le mode position. \
Il sera bientôt disponible donc: stay tuned.

## il se peut que vous ayez à refaire l'installation du PX4 manuellement ainsi que l'ajout du modèle x500
#faites ceci seulement si vous aviez une erreur du type "gazebo-classic not found " à la fin du tuto lors du lancement du px4
##
cd \
git clone --recursive https://github.com/PX4/PX4-Autopilot.git \
cd PX4-Autopilot \
git checkout v1.15.0-beta1-703-gda8827883f 

cd \
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh \
cd PX4-Autopilot/ \
make px4_sitl -j$(nproc)\
### pensez à saisir y puis valider avec ENTREE  pour accepter les modifications
sudo apt remove gz-harmonic \
sudo apt install gazebo libgazebo11 libgazebo-dev \
cd 
cd gazebo-classic_x500
./add_gazebo-classic_x500.sh /root

cd
cp x500.sdf /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/x500




#FIN









