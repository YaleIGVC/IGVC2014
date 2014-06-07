sudo apt-get install subversion libwxgtk2.8-dev liblivemedia-dev
cd /opt/
sudo svn co https://svn.code.sf.net/p/active-ist/code/trunk active-ist
cd active-ist/
sudo sed -i '54i#include <cstring>' src/cxutils/Packet.h
sudo cmake -DBUILD_JAUS_ONLY=True .
sudo make
