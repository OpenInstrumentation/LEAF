# LEAF
Logger for Environmental and Atmospheric Factors

CC-BY-SA 2019, A. Matheny, P. Marchetto, J. Powell, A. Rechner, J-y. Chuah, and S. Pierce

BoM: https://docs.google.com/spreadsheets/d/e/2PACX-1vQ8v-ukr38iTDn-cAedqp6OXnIay0lwoVsFbz_EMlWamA66PgREalaegVoSEzRlPvawVt73zznOhLpU/pubhtml

# Purpose
The LEAF is designed to be a modular sensing system, capable of measuring meteorological, hydrological, and other environmental parameters. In the design detailed in this repository, it was used to measure canopy throughfall of rain, soil moisture, and microclimate meteorological parameters, along with branch movement in the same tree the throughfall measurements were being made in.

# Files
The files in this repository are what's needed to program the different nodes of the sensing system network. The ground .ino file is different from the branches because it measures soil moisture as well; both of the .ino files are designed to be uploaded via Particle's online IDE at http://build.particle.io/. The catalog file includes all of the different items as priced at the time of the build, and the BoM includes direct links to them. The deployment file shows roughly what the initial instance looked like in its first deployment. The .brd and .sch files are Eagle board and schematic files, respectively, used to make PCBs for the system.
