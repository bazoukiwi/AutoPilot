# AutoPilot
Arduino code for a simple boat autopilot 


Fonctionnement

L'idée du projet est de reproduire le fonctionnement d'un pilote automatique classique. Le pilote doit essayer d'amener le bateau sur le cap défini par l'utilisateur. 
L'utilisateurs peut acceder aux commandes suivantes : 
- Standby : Stoppe toute activité du pilote
- Auto : demande au pilote de suivre le cap actuel du bateau 
- +1/+10/-1/-10 : modifie le cap que le pilote doit suivre
- Mode GPS / Compas : spécifie si le pilote doit suivre un cap GPS ou magnétique
- Gain : Multiplicateur d'angle de barre à appliquer (potentiometre) 
- Level : Multiplicateur de delta d'erreur

La carte arduino recoit les commandes par une telecommande infrarouge et affiche les settings et informations sur un ecran LCD. 
Elle reçoit les données GPS / Compas via un module GPS ainsi qu'une centrale intertielle IMU 9 axes. 
Elle essaye enfin d'appliquer les ordres en envoyant des impulsions PWM a un pont en H auquel est relié le moteur du vérin de barre. 

Connexions :
Infrared Receiver : PIN 7
LCD : Interface I2C, adresse 0x27
IMU (inertielle) : Interface I2C,adresse 0x68
IBT-2 (H bridge) : PIN 5 & 6 
GPS : Serial Tx Rx

Calibration : Pour activer la calibration, passer la ligne 65 a true puis copier les valeurs obtenues dans le code juste en dessous. 
Sans cela, le cap (HDG) sera completement faux. 

Limitations : 
La centrale inertielle etant plutot vétuste, les résultats obtenus sont faussés à une gite excessive (Fonction ligne 765 qui compute les angles). 
Pour résoudre ce probleme, il faudrai utiliser les equations quaternions mais c'est en dehors de ma portée. 

Le pilote repose sur le princide de PID : 
https://fr.wikipedia.org/wiki/Régulateur_PID





