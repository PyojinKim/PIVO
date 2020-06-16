# Patch-based Illumination invariant Direct Visual Odometry (PIVO)
This package provides a MATLAB implementation of AURO 2019 & IROS 2015 papers: "Autonomous Flight with Robust Visual Odometry under Dynamic Lighting Conditions" & "Robust Visual Odometry to Irregular Illumination Changes with RGB-D camera" for the purpose of research and study only.

![PIVO](https://github.com/PyojinKim/PIVO/blob/master/teaser.png)


# 1. License
The package is licenced under the MIT License, see http://opensource.org/licenses/MIT.

if you use PIVO in an academic work, please cite:

    @inproceedings{kim2017visual,
	  author = {Kim, Pyojin and Coltin, Brian and Kim, H Jin},
	  title = {Visual Odometry with Drift-Free Rotation Estimation Using Indoor Scene Regularities},
      year = {2017},
	  booktitle = {British Machine Vision Conference (BMVC)},
     }

	 
# 2. Prerequisites


# 3. Usage
* Download the ICL-NUIM dataset from https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html, 'of kt3' is recommanded.

* Or, Use the ICL-NUIMdataset/of_kt3/ included in this package.

* Define 'datasetPath' correctly in your directory at setupParams_ICL_NUIM.m file.

* Run OPVO_core/main_script_ICL_NUIM.m which will give you the 3D motion estimation result. Enjoy! :)


