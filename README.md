# AudioClassification


Clone the repo into a desired location. In this example we will use the local Desktop and install the module. 
## Installation:
    cd ~/Desktop
    git clone https://github.com/hoergems/AudioClassification.git
    mkdir Observation && cd Observation
    mkdir src
    ln -s ~/Desktop/AudioClassification/ObservationService ~/Desktop/Observation/src
    catkin_make
    cd ~/Desktop
    cd AudioClassification && mkdir build && cd build
    source ~/Desktop/Observation/devel/setup.bash
    cmake -DCMAKE_INSTALL_PREFIX=<install folder> ..
    make && make install

## Usage
On the MOVO2 computer, open a terminal and run

    movostop

Then run

    roslaunch movo_bringup_simple main.launch

On your computer, open a terminal and run

    source <install folder>/share/oppt/setup.sh

or add this line to your .bashrc file. \<install folder\> is the installation folder specified in the cmake command above. In the same terminal you can then run the problem with the provided config file, e.g.

    cd <oppt folder>/bin
    ./abt --cfg <folder where this repo is cloned into>/cfg/AudioClassification.cfg

![Alt text](block_diagram.jpeg)
