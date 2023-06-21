## XL_320Toolbox
========

#####Matlab Toolbox to control ROBOTIS Dynamixel XL_320 smart servo actuators with the OpenCM9.04 microcontroller.#####
######Version 1.0, 2-12-17######
Methods had been created by Dr. Federico Parietti for Dynamixel Pro

--------

First of all you will need to download the DXLTosser on the OpenCM9.04 

In ROBOTIS-v1.0.4 windows version Go to menu File > Example > DYNAMIXEL > Tosser

Download ROBOTIS software for OpenCM here http://support.robotis.com/en/software/robotis_opencm/robotis_opencm.htm

--------

Hamid Osooli, hamid.osooli@gmail.com

Created: 11-02-17

This version tested with MATLAB R2015a  

--------

## Instructions

Add the XL_320Toolbox folder to the MATLAB path.

Open the file “XL_320_Example.m” to see some examples showing how to use the library functions to
write and read instructions to/from the Dynamixel XL_320 servos.

Thanks to the work by Dr. Federico Parietti All functions are extensively commented.

Notice that the library contains two kind of functions:
-	simple Write and Read functions: to control one Dynamixel XL_320  servo at a time; multiple servos can
still be controlled by the same code (by accessing them one by one), but this quickly increases computing
time;
-	Sync_Write and Sync-Read functions: to control multiple Dynamixel XL_320 servos at the same time; these
functions are a little more complicated, but they are essential if you need to control many servos with a faster sampling rate. 

--------

## License

This work is provided with a GNU GPL v3.0 license (see attached file).

Remember to credit the author of the library when using this work.

## Citation

If you used XL-320 toolbox in your project, we would really appreciate if you could cite our work:

- [1] Hamid Osooli, Mohsen Irani Rahaghi, S. Reza Ahmadzadeh, "Design and Evaluation of a Bioinspired Tendon-Driven 3D-Printed Robotic Eye with Active Vision Capabilities," In Proc.  20th International Conference on Ubiquitous Robots (UR 2023), Honolulu, Hawaii, pp. xxx-xxx, Jun. 25-28, 2023

```bibtex
@article{osooli2023design,
  title={Design and Evaluation of a Bioinspired Tendon-Driven 3D-Printed Robotic Eye with Active Vision Capabilities},
  author={Osooli, Hamid and Rahaghi, Mohsen Irani and Ahmadzadeh, S Reza},
  journal={arXiv preprint arXiv:2305.01076},
  year={2023}
}

```

- [2] Hamid Osooli, Amirhossein Nikoofard, and Zahra Shirmohammadi. "Game Theory for Eye Robot Movement: Approach and Hardware Implementation." In 2019 27th Iranian Conference on Electrical Engineering (ICEE 2019), Yazd, Iran, pp. 1081-1085. IEEE, 2019, DOI: 10.1109/IranianCEE.2019.8786637

```bibtex
@inproceedings{osooli2019game,
  title={Game Theory for Eye Robot Movement: Approach and Hardware Implementation},
  author={Osooli, Hamid and Nikoofard, Amirhossein and Shirmohammadi, Zahra},
  booktitle={2019 27th Iranian Conference on Electrical Engineering (ICEE)},
  pages={1081--1085},
  year={2019},
  organization={IEEE},
  doi={10.1109/IranianCEE.2019.8786637}
}

```

