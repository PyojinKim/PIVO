# Patch-based Illumination invariant Direct Visual Odometry (PIVO)
This package provides a MATLAB implementation of AURO 2019 & IROS 2015 papers: "Autonomous Flight with Robust Visual Odometry under Dynamic Lighting Conditions" & "Robust Visual Odometry to Irregular Illumination Changes with RGB-D camera" for the purpose of research and study only.
Note that this repository only includes simplified proposed direct visual odometry example codes to understand how the PIVO works under light-changing environments.

![PIVO](https://github.com/PyojinKim/PIVO/blob/master/teaser.png)


# 1. License
The package is licensed under the MIT License, see http://opensource.org/licenses/MIT.

if you use PIVO in an academic work, please cite:

    @inproceedings{kim2019autonomous,
	  author = {Kim, Pyojin and Lee, Hyeonbeom and Kim, H Jin},
	  title = {Autonomous Flight with Robust Visual Odometry under Dynamic Lighting Conditions},
      year = {2019},
	  booktitle = {Autonomous Robots (AURO)},
     }
	 
	@inproceedings{kim2015robust,
	  author = {Kim, Pyojin and Lim, Hyon and Kim, H Jin},
      title = {Robust Visual Odometry to Irregular Illumination Changes with RGB-D camera},
      year = {2015},
      booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
	 }
	 
	
# 2. Prerequisites
This package is tested on the MATLAB R2019b on Windows 7 64-bit.
Some of the functions such as warpEntireImage_mex.mexw64 are compiled as MEX file to speed up the computation.
You can use warpEntireImage.m instead if you cannot compile MEX file.


# 3. Usage
* Download the ICL-NUIM dataset from https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html, 'of kt3' is recommended.

* Or, Use the ICL-NUIMdataset/of_kt3/ included in this package.

* Define 'datasetPath' correctly in your directory at setupParams_ICL_NUIM.m file.

* Run PIVO_core/main_script_ICL_NUIM.m, which will give you the 3D motion estimation result. Enjoy! :)


# 4. Publications
The approach is described and used in the following publications:

* **Autonomous Flight with Robust Visual Odometry under Dynamic Lighting Conditions** (Pyojin Kim, Hyeonbeom Lee, and H. Jin Kim), AURO 2019.

* **Robust Visual Odometry to Irregular Illumination Changes with RGB-D camera** (Pyojin Kim, Hyon Lim, and H. Jin Kim), IROS 2015.

You can find more related papers at http://pyojinkim.com/_pages/pub/index.html.
