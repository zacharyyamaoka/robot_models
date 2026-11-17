"""Where to learn each rung's model (PEP 0004): real, citable works for the type ladders to cite.

Copied on 2026-11-16 from the references sim2real's type cards cite, where every DOI was resolved at
Crossref with its title matched and every URL fetched. Works with no stable URL or DOI there (Brown 1966,
the ArUco paper, Triggs 2000, Strobl 2011) are cited without one. Left out, by PEP 0004 decision 4 (type
data only): a maker's manual or page, and one rig's own code. A ladder cites them as
`references=(ARMSTRONG_1994, OLSSON_1998)`. Nothing here is a value.
"""

from __future__ import annotations

from .ladder import Reference


def _doi(title: str, doi: str, note: str = "", kind: str = "paper") -> Reference:
    return Reference(title, url=f"https://doi.org/{doi}", kind=kind, doi=doi, note=note)


# --- friction, backlash, ripple, gears ----------------------------------------------------------------------
ARMSTRONG_1994 = _doi(
    "Armstrong-Hélouvry, Dupont, Canudas de Wit (1994). A survey of models, analysis tools and "
    "compensation methods for the control of machines with friction. Automatica.",
    "10.1016/0005-1098(94)90209-7",
    "Coulomb, viscous and Stribeck friction",
)
OLSSON_1998 = _doi(
    "Olsson, Åström, Canudas de Wit, Gäfvert, Lischinsky (1998). Friction models and friction "
    "compensation. European Journal of Control.",
    "10.1016/S0947-3580(98)70113-X",
    "static friction models from Coulomb + viscous to Stribeck",
)
CANUDAS_1995 = _doi(
    "Canudas de Wit, Olsson, Åström, Lischinsky (1995). A new model for control of systems with "
    "friction. IEEE Transactions on Automatic Control.",
    "10.1109/9.376053",
    "LuGre: the dynamic friction model robotparts' M3.lugre is",
)
KIRCANSKI_1997 = _doi(
    "Kircanski, Goldenberg (1997). An experimental study of nonlinear stiffness, hysteresis, and "
    "friction effects in robot joints with harmonic drives and torque sensors. The International "
    "Journal of Robotics Research.",
    "10.1177/027836499701600207",
    "load-dependent friction and torsional stiffness measured in a joint",
)
NORDIN_2002 = _doi(
    "Nordin, Gutman (2002). Controlling mechanical systems with backlash—a survey. Automatica.",
    "10.1016/S0005-1098(02)00047-X",
    "dead-zone backlash between motor and load",
)
TUTTLE_1996 = _doi(
    "Tuttle, Seering (1996). A nonlinear model of a harmonic drive gear transmission. IEEE "
    "Transactions on Robotics and Automation.",
    "10.1109/70.499819",
    "a gear's compliance, friction and kinematic error",
)
JAHNS_1996 = _doi(
    "Jahns, Soong (1996). Pulsating torque minimization techniques for permanent magnet AC motor "
    "drives—a review. IEEE Transactions on Industrial Electronics.",
    "10.1109/41.491356",
    "cogging and commutation ripple as rotor harmonics",
)

# --- motors and drives --------------------------------------------------------------------------------------
KRAUSE_2013 = _doi(
    "Krause, Wasynczuk, Sudhoff, Pekarek (2013). Analysis of Electric Machinery and Drive Systems,"
    " 3rd ed. Wiley-IEEE Press.",
    "10.1002/9781118524336",
    "the PM machine's electrical model: R, L, back-EMF, the voltage ceiling",
    kind="book",
)
KRISHNAN_2010 = _doi(
    "Krishnan (2010). Permanent Magnet Synchronous and Brushless DC Motor Drives. CRC Press.",
    "10.1201/9781420014235",
    "field-oriented current loops of PM drives",
    kind="book",
)

# --- rigid bodies, chains and their calibration -------------------------------------------------------------
SICILIANO_2009 = _doi(
    "Siciliano, Sciavicco, Villani, Oriolo (2009). Robotics: Modelling, Planning and Control. "
    "Springer.",
    "10.1007/978-1-84628-642-1",
    "rigid-body joint dynamics and reflected inertia",
    kind="book",
)
DH_1955 = _doi(
    "Denavit, Hartenberg (1955). A kinematic notation for lower-pair mechanisms based on matrices."
    " Journal of Applied Mechanics.",
    "10.1115/1.4011045",
    "the DH parameters of a serial chain",
)
HAYATI_1983 = _doi(
    "Hayati (1983). Robot arm geometric link parameter estimation. The 22nd IEEE Conference on "
    "Decision and Control.",
    "10.1109/CDC.1983.269783",
    "modified DH for parallel axes, the form kinematic calibration fits",
)
OKAMURA_1996 = _doi(
    "Okamura, Park (1996). Kinematic calibration using the product of exponentials formula. "
    "Robotica.",
    "10.1017/S0263574700019810",
    "a singularity-free calibration parameterisation",
)
HOLLERBACH_WAMPLER_1996 = _doi(
    "Hollerbach, Wampler (1996). The calibration index and taxonomy for robot kinematic "
    "calibration methods. The International Journal of Robotics Research.",
    "10.1177/027836499601500604",
    "open- and closed-loop kinematic calibration",
)
HOLLERBACH_2016 = _doi(
    "Hollerbach, Khalil, Gautier (2016). Model identification. In Springer Handbook of Robotics, "
    "2nd ed. Springer.",
    "10.1007/978-3-319-32552-1_6",
    "kinematic calibration and inertial identification, one chapter",
    kind="chapter",
)
ATKESON_1986 = _doi(
    "Atkeson, An, Hollerbach (1986). Estimation of inertial parameters of manipulator loads and "
    "links. The International Journal of Robotics Research.",
    "10.1177/027836498600500306",
    "link and load inertials by least squares",
)
GAUTIER_1990 = _doi(
    "Gautier, Khalil (1990). Direct calculation of minimum set of inertial parameters of serial "
    "robots. IEEE Transactions on Robotics and Automation.",
    "10.1109/70.56655",
    "the base inertial parameters an arm can identify",
)
SWEVERS_1997 = _doi(
    "Swevers, Ganseman, Tükel, De Schutter, Van Brussel (1997). Optimal robot excitation and "
    "identification. IEEE Transactions on Robotics and Automation.",
    "10.1109/70.631234",
    "periodic excitation trajectories for dynamic identification",
)
SPONG_1987 = _doi(
    "Spong (1987). Modeling and control of elastic joint robots. Journal of Dynamic Systems, "
    "Measurement, and Control.",
    "10.1115/1.3143860",
    "joint compliance as a spring between motor and link",
)
PRATTICHIZZO_2016 = _doi(
    "Prattichizzo, Trinkle (2016). Grasping. In Springer Handbook of Robotics, 2nd ed. Springer.",
    "10.1007/978-3-319-32552-1_38",
    "contact, friction cones and grasp force",
    kind="chapter",
)

# --- belts --------------------------------------------------------------------------------------------------
ZHANG_XIA_2011 = _doi(
    "Zhang, Xia (2011). Modeling and energy efficiency optimization of belt conveyors. Applied "
    "Energy.",
    "10.1016/j.apenergy.2011.03.015",
    "a belt conveyor's resistances and drive power",
)
HE_2017 = _doi(
    "He, Pang, Lodewijks (2017). Green operations of belt conveyors by means of speed control. "
    "Applied Energy.",
    "10.1016/j.apenergy.2016.12.017",
    "a belt conveyor's dynamics under speed changes",
)

# --- cameras and markers ------------------------------------------------------------------------------------
ZHANG_2000 = _doi(
    "Zhang (2000). A flexible new technique for camera calibration. IEEE Transactions on Pattern "
    "Analysis and Machine Intelligence.",
    "10.1109/34.888718",
    "pinhole intrinsics from a planar target in several poses",
)
BROWN_1966 = Reference(
    "Brown (1966). Decentering distortion of lenses. Photogrammetric Engineering.",
    note="the radial and tangential (Brown-Conrady) distortion model; no stable URL",
)
HEIKKILA_1997 = _doi(
    "Heikkilä, Silvén (1997). A four-step camera calibration procedure with implicit image "
    "correction. IEEE Conference on Computer Vision and Pattern Recognition.",
    "10.1109/CVPR.1997.609468",
    "pinhole + radial and tangential distortion fitted together",
)
OTH_2013 = _doi(
    "Oth, Furgale, Kneip, Siegwart (2013). Rolling shutter camera calibration. IEEE Conference on "
    "Computer Vision and Pattern Recognition.",
    "10.1109/CVPR.2013.179",
    "the line delay of a rolling shutter, calibrated",
)
MEINGAST_2005 = Reference(
    "Meingast, Geyer, Sastry (2005). Geometric models of rolling-shutter cameras. "
    "arXiv:cs/0503076.",
    url="https://arxiv.org/abs/cs/0503076",
    kind="preprint",
    note="each row exposed at its own time",
)
SCHOPS_2020 = _doi(
    "Schöps, Larsson, Pollefeys, Sattler (2020). Why having 10,000 parameters in your camera model"
    " is better than twelve. IEEE/CVF Conference on Computer Vision and Pattern Recognition.",
    "10.1109/CVPR42600.2020.00261",
    "a dense B-spline generic camera model above the parametric one",
)
KHOSHELHAM_2012 = _doi(
    "Khoshelham, Elberink (2012). Accuracy and resolution of Kinect depth data for indoor mapping "
    "applications. Sensors.",
    "10.3390/s120201437",
    "depth from disparity and its error growing with z²",
)
HERRERA_2012 = _doi(
    "Herrera C., Kannala, Heikkilä (2012). Joint depth and color camera calibration with "
    "distortion correction. IEEE Transactions on Pattern Analysis and Machine Intelligence.",
    "10.1109/TPAMI.2012.125",
    "depth bias as a spatially varying disparity distortion",
)
GARRIDO_2014 = Reference(
    "Garrido-Jurado, Muñoz-Salinas, Madrid-Cuevas, Marín-Jiménez (2014). Automatic generation and "
    "detection of highly reliable fiducial markers under occlusion. Pattern Recognition "
    "47(6):2280-2292.",
    note="the ArUco dictionaries, their ids and the detector OpenCV ships",
)
TRIGGS_2000 = Reference(
    "Triggs, McLauchlan, Hartley, Fitzgibbon (2000). Bundle adjustment: a modern synthesis. Vision"
    " Algorithms: Theory and Practice, LNCS 1883, Springer.",
    kind="chapter",
    note="robust bundle adjustment, priors and the gauge (one sticker held at CAD)",
)
STROBL_2011 = Reference(
    "Strobl, Hirzinger (2011). More accurate pinhole camera calibration with imperfect planar "
    "target. IEEE International Conference on Computer Vision Workshops.",
    note="a target's own geometry (scale, flatness) fitted with the camera",
)

# --- spectral imaging ---------------------------------------------------------------------------------------
BURGER_2011 = _doi(
    "Burger, Gowen (2011). Data handling in hyperspectral image analysis. Chemometrics and "
    "Intelligent Laboratory Systems.",
    "10.1016/j.chemolab.2011.04.001",
    "dark and white reference reflectance calibration",
)
KRUSE_1993 = _doi(
    "Kruse, Lefkoff, Boardman, Heidebrecht, Shapiro, Barloon, Goetz (1993). The spectral image "
    "processing system (SIPS)—interactive visualization and analysis of imaging spectrometer data."
    " Remote Sensing of Environment.",
    "10.1016/0034-4257(93)90013-N",
    "the spectral angle",
)
MOUROULIS_2000 = _doi(
    "Mouroulis, Green, Chrien (2000). Design of pushbroom imaging spectrometers for optimum "
    "recovery of spectroscopic and spatial information. Applied Optics.",
    "10.1364/AO.39.002210",
    "smile and keystone",
)
GUPTA_1997 = _doi(
    "Gupta, Hartley (1997). Linear pushbroom cameras. IEEE Transactions on Pattern Analysis and "
    "Machine Intelligence.",
    "10.1109/34.615446",
    "the line-scan (pushbroom) camera's geometry",
)

# --- encoders -----------------------------------------------------------------------------------------------
WIDROW_2008 = _doi(
    "Widrow, Kollár (2008). Quantization Noise. Cambridge University Press.",
    "10.1017/CBO9780511754661",
    "an encoder as a quantiser",
    kind="book",
)
MERRY_2010 = _doi(
    "Merry, van de Molengraft, Steinbuch (2010). Velocity and acceleration estimation for optical "
    "incremental encoders. Mechatronics.",
    "10.1016/j.mechatronics.2009.06.010",
    "encoder noise, quantisation and delay in velocity",
)
ZHAO_2024 = _doi(
    "Zhao, Wan, Liang (2024). Automatic compensation system for eccentricity error of absolute "
    "optical encoder. Review of Scientific Instruments.",
    "10.1063/5.0211297",
    "eccentricity as once-per-turn harmonics",
)

# --- wear and degradation -----------------------------------------------------------------------------------
ARCHARD_1953 = _doi(
    "Archard (1953). Contact and rubbing of flat surfaces. Journal of Applied Physics.",
    "10.1063/1.1721448",
    "wear volume grows with the load carried times the distance slid",
)
FLODIN_1997 = _doi(
    "Flodin, Andersson (1997). Simulation of mild wear in spur gears. Wear.",
    "10.1016/S0043-1648(96)07467-4",
    "Archard's law on gear teeth: the flank wear that opens backlash",
)
LU_MEEKER_1993 = _doi(
    "Lu, Meeker (1993). Using degradation measures to estimate a time-to-failure distribution. "
    "Technometrics.",
    "10.1080/00401706.1993.10485038",
    "a degradation path in usage, linear the simplest, crossing a failure threshold",
)
MEEKER_1998 = _doi(
    "Meeker, Escobar, Lu (1998). Accelerated degradation tests: modeling and analysis. "
    "Technometrics.",
    "10.1080/00401706.1998.10485191",
    "degradation paths fitted and extrapolated to a threshold",
)
WHITMORE_1995 = _doi(
    "Whitmore (1995). Estimating degradation by a Wiener diffusion process subject to measurement "
    "error. Lifetime Data Analysis.",
    "10.1007/BF00985762",
    "diffusive drift: its expected size grows as the square root of time",
)
JUNKER_1969 = _doi(
    "Junker (1969). New criteria for self-loosening of fasteners under vibration. SAE Technical "
    "Paper.",
    "10.4271/690055",
    "a mount loosening under transverse vibration",
)

# --- residuals and learned nets -----------------------------------------------------------------------------
NGUYEN_TUONG_2010 = _doi(
    "Nguyen-Tuong, Peters (2010). Using model knowledge for learning inverse dynamics. IEEE "
    "International Conference on Robotics and Automation.",
    "10.1109/ROBOT.2010.5509858",
    "a physics model plus a learned residual",
)
ZENG_2020 = _doi(
    "Zeng, Song, Lee, Rodriguez, Funkhouser (2020). TossingBot: learning to throw arbitrary "
    "objects with residual physics. IEEE Transactions on Robotics.",
    "10.1109/TRO.2020.2988642",
    "residual physics: learn what the physics estimate misses",
)
KOVALEV_2026 = Reference(
    "Kovalev, Chaikovskaia, Davydenko, Gorbachev et al. (2026). Trajectory-based actuator "
    "identification via differentiable simulation. arXiv:2604.10351.",
    url="https://arxiv.org/abs/2604.10351",
    kind="preprint",
    note="a structured servo fit vs an MLP vs a residual model on held-out position error",
)
HWANGBO_2019 = _doi(
    "Hwangbo, Lee, Dosovitskiy, Bellicoso, Tsounis, Koltun, Hutter (2019). Learning agile and "
    "dynamic motor skills for legged robots. Science Robotics.",
    "10.1126/scirobotics.aau5872",
    "ANYmal's actuator net: an MLP over position error and velocity at t, t − 10 ms, t − 20 ms, 3 "
    "× 32 softsign (arXiv:1901.08652)",
)
HWANGBO_2019_ARXIV = Reference(
    "Hwangbo, Lee, Dosovitskiy, Bellicoso, Tsounis, Koltun, Hutter (2019). Learning agile and "
    "dynamic motor skills for legged robots. arXiv:1901.08652.",
    url="https://arxiv.org/abs/1901.08652",
    kind="preprint",
    note="the open version",
)
ZHU_2025 = Reference(
    "Zhu, Tanaka, Rafeedi, Hong (2025). Cycloidal quasi-direct drive actuator designs with "
    "learning-based torque estimation for legged robotics. arXiv:2410.16591.",
    url="https://arxiv.org/abs/2410.16591",
    kind="preprint",
    note="a longer history (a 30-step GRU) when ripple or backlash matter",
)

# --- simulator documentation --------------------------------------------------------------------------------
MUJOCO_ARMATURE = Reference(
    "MuJoCo (2026). XML reference: joint armature.",
    url="https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-armature",
    kind="doc",
    note="the rotor's inertia reflected through the gear, as the simulator takes it",
)
MUJOCO_DCMOTOR = Reference(
    "MuJoCo (2026). XML reference: the dcmotor actuator.",
    url="https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-dcmotor",
    kind="doc",
    note="robotparts' M5.dcmotor: electrics, heat, cogging and LuGre as MuJoCo takes them",
)
MUJOCO_CONTACT = Reference(
    "MuJoCo (2026). Computation: the contact model.",
    url="https://mujoco.readthedocs.io/en/stable/computation/index.html",
    kind="doc",
    note="soft contact with friction cones, what the twin's grasp runs on",
)

ALL = (
    ARMSTRONG_1994,
    OLSSON_1998,
    CANUDAS_1995,
    KIRCANSKI_1997,
    NORDIN_2002,
    TUTTLE_1996,
    JAHNS_1996,
    KRAUSE_2013,
    KRISHNAN_2010,
    SICILIANO_2009,
    DH_1955,
    HAYATI_1983,
    OKAMURA_1996,
    HOLLERBACH_WAMPLER_1996,
    HOLLERBACH_2016,
    ATKESON_1986,
    GAUTIER_1990,
    SWEVERS_1997,
    SPONG_1987,
    PRATTICHIZZO_2016,
    ZHANG_XIA_2011,
    HE_2017,
    ZHANG_2000,
    BROWN_1966,
    HEIKKILA_1997,
    OTH_2013,
    MEINGAST_2005,
    SCHOPS_2020,
    KHOSHELHAM_2012,
    HERRERA_2012,
    GARRIDO_2014,
    TRIGGS_2000,
    STROBL_2011,
    BURGER_2011,
    KRUSE_1993,
    MOUROULIS_2000,
    GUPTA_1997,
    WIDROW_2008,
    MERRY_2010,
    ZHAO_2024,
    ARCHARD_1953,
    FLODIN_1997,
    LU_MEEKER_1993,
    MEEKER_1998,
    WHITMORE_1995,
    JUNKER_1969,
    NGUYEN_TUONG_2010,
    ZENG_2020,
    KOVALEV_2026,
    HWANGBO_2019,
    HWANGBO_2019_ARXIV,
    ZHU_2025,
    MUJOCO_ARMATURE,
    MUJOCO_DCMOTOR,
    MUJOCO_CONTACT,
)
