"""File containing the WPT system class."""

import numpy as np

"""
A WPT system is divided in 3 class : 
* the transmitter (with its topology and componants value)
* the reciever (with its topology and componants value)
* the total system (based on a transmitter, a reciever and a mutual inductance)
"""


class transmitter:
    """Transmitter system class."""

    def __init__(
        self,
        topology: str = "S",
        L: float = None,
        C_s: float = None,
        C_p: float = None,
        R: float = None,
        L_s: float = None,
        height: float = None,
        width: float = None,
        N: int = None,
    ) -> None:
        """Class constructor.

        Args:
            topology (str): transmitter topology, it can be 'S', 'P', 'SP' or 'LCC'. Defaults to 's'.
            L (float): transmitter coil inductance in Henry. Defaults to None.
            C_s (float): Serie compensation capacitor value in Farad. Defaults to None.
            C_p (float): Parallel compensation capacitor value in Farad. Defaults to None.
            R (float): Internal transmitter of the reciever coil in Ohm. Delaults to None.
            L_s (float): Serie inductance used in the LCC topology. Value in Henry. Defaults to None.
            height (float) : Height of the coil in meter. Delaults to None.
            width (float) : Width of the coil in meter. Delaults to None.
            N (int) : Number of turn in the coil. Delaults to None.
        """

        if topology not in ["S", "P", "SP", "LCC"]:
            raise ValueError(
                f"invalid topology, {topology} was entered, should 'S', 'P' or 'SP' "
            )

        self.topology = topology
        self.L = L
        self.C_s = C_s
        self.C_p = C_p
        self.R = R
        self.L_s = L_s
        self.height = height
        self.width = width
        self.N = N


class reciever:
    """Reciever system class.

    The reciever is the system on the car composed of a coil, compensation circuit,
    """

    def __init__(
        self,
        topology: str = "S",
        L: float = None,
        R: float = None,
        C_s: float = None,
        C_p: float = None,
        R_l: float = None,
        L_s: float = None,
        height: float = None,
        width: float = None,
        N: int = None,
    ) -> None:
        """Class constructor.

        Args:
            tytopology (str): Reciever topology, it can be 'S', 'P' or 'LCC'. Defaults to 's'.
            L (float): Reciever coil inductance in Henry. Defaults to None.
            R (float): Internal resistance of the reciever coil in Ohm. Delaults to None.
            C_s (float): Serie compensation capacitor value in Farad. Defaults to None.
            C_p (float): Parallel compensation capacitor value in Farad. Defaults to None.
            L_s (float): Serie inductance used in the LCC topology. Value in Henry. Defaults to None.
            R_l (float): Resistance Load in Ohm. Defaults to None.
            height (float) : Height of the coil in meter. Defaults to None.
            width (float) : Width of the coil in meter. Defaults to None.
            N (int) : Number of turn in the coil. Defaults to None.
        """

        if topology not in ["S", "P", "LCC"]:
            raise ValueError(
                f"invalid topology, {topology} was entered, should 'S' or 'P' "
            )

        self.topology = topology
        self.L = L
        self.C_s = C_s
        self.C_p = C_p
        self.L_s = L_s
        self.R_l = R_l
        self.R = R
        self.height = height
        self.width = width
        self.N = N


class total_system:

    """Total system class.

    A total system is composed of a transmitter, a reciever and the mutual inductance between the two
    or the parameters that are necessary to compute the mutual inductances.
    """

    def __init__(
        self,
        transmitter: transmitter = None,
        reciever: reciever = None,
        name: str = None,
        M: float = 0,
        M_from_geo: bool = False,
        h: float = None,
    ) -> None:
        """Class constructor.

        Args:
            transmitter (transmitter): transmitter object used to create the system. Defaults to None.
            reciever (reciever): reciever object used to create the system. Defaults to None.
            M (float, optional): Mutual inductance between the 2 coils in Henry if not calculated. Defaults to 0.
            name (str) : name of the WPT system. Defaults to None.
            M_from_geo (bool, optional): Bool used to choose if the mutual inductance needs to be computed from
                coils geometry or taken as a parameter. Defaults to False.
            h (float, optional): disance between the coils in meter. Defaults to None.

        Raises:
            ValueError: In case of incorect topology
        """
        self.name = name
        self.topology = transmitter.topology + "-" + reciever.topology

        self.transmitter = transmitter
        self.reciever = reciever
        self.h = h

        if M_from_geo:
            self.M = self._mutual_from_geometry()
        else:
            self.M = M

        # Select the impedance function according to the topology

        match self.topology:
            case "S-S":
                self.impedance = self._S_S_impedance
            case "S-P":
                self.impedance = self._S_P_impedance
            case "P-S":
                self.impedance = self._P_S_impedance
            case "P-P":
                self.impedance = self._P_P_impedance
            case "SP-S":
                self.impedance = self._SP_S_impedance
            case "SP-P":
                self.impedance = self._SP_P_impedance
            case "LCC-LCC":
                self.impedance = self._LCC_LCC_impedance
            case "LCC-S":
                self.impedance = self._LCC_S_impedance
            case _:
                raise ValueError(
                    f"invalid topology, transmitter : {self.transmitter.topology} , reciever : {self.reciever.topology}"
                )

    def bode_plot_impedance(
        self, min_freq: float = 1, max_freq: float = 500000, nb_samples: int = 200
    ) -> [list, list, list]:
        gain_dB, phase = [], []
        freqs = np.linspace(min_freq, max_freq, nb_samples)
        # print(f"topology : {self.topology}, impedance at f0 : {self.impedance(85000)}")
        for freq in freqs:
            impedance_at_freq = self.impedance(freq)
            gain_dB.append(20 * np.log10(np.absolute(impedance_at_freq)))
            phase.append(np.angle(impedance_at_freq) * 180 / np.pi)

        return gain_dB, phase, freqs

    # You can check "Wireless Power Transfer Structure Design for Electric Vehicle in Charge While Driving"
    #  By V. Cirimele & al for the following formulas of the inductance

    def _S_S_impedance(self, frequency: float = None) -> complex:
        """Function computing the equivalent impedance for a S-S topology WPT system.

        Args:
            frequency (float): Frequency at wich the impedance must be computed. Defaults to None.

        Returns:
            complex: Equivalent impedance of the system viewed by the source at the frequency past in argument.
        """

        R1 = self.transmitter.R
        L1 = self.transmitter.L
        C1 = self.transmitter.C_s
        M = self.M
        R2 = self.reciever.R
        L2 = self.reciever.L
        C2 = self.reciever.C_s
        R_l = self.reciever.R_l
        w = 2 * np.pi * frequency

        return (
            R1
            + 1j * w * L1
            + 1 / (1j * w * C1)
            + (w**2 * M**2) / (R2 + R_l + 1j * w * L2 + 1 / (1j * w * C2))
        )

    def _S_P_impedance(self, frequency: float = None) -> complex:
        """Function computing the equivalent impedance for a S-P topology WPT system.

        Args:
            frequency (float): Frequency at wich the impedance must be computed. Defaults to None.

        Returns:
            complex: Equivalent impedance of the system viewed by the source at the frequency past in argument.
        """

        R1 = self.transmitter.R
        L1 = self.transmitter.L
        C1 = self.transmitter.C_s
        M = self.M
        R2 = self.reciever.R
        L2 = self.reciever.L
        C2 = self.reciever.C_p
        R_l = self.reciever.R_l
        w = 2 * np.pi * frequency

        return (
            R1
            + 1j * w * L1
            + 1 / (1j * w * C1)
            + (w**2 * M**2) / (R2 + 1j * w * L2 + R_l / (1 + 1j * w * R_l * C2))
        )

    def _P_S_impedance(self, frequency: float = None) -> complex:
        """Function computing the equivalent impedance for a P-S topology WPT system.

        Args:
            frequency (float): Frequency at wich the impedance must be computed. Defaults to None.

        Returns:
            complex: Equivalent impedance of the system viewed by the source at the frequency past in argument.
        """

        R1 = self.transmitter.R
        L1 = self.transmitter.L
        C1 = self.transmitter.C_p
        M = self.M
        R2 = self.reciever.R
        L2 = self.reciever.L
        C2 = self.reciever.C_s
        R_l = self.reciever.R_l
        w = 2 * np.pi * frequency

        return (
            1
            / (
                R1
                + 1j * w * L1
                + (w**2 * M**2) / (R2 + R_l + 1j * w * L2 + 1 / (1j * w * C2))
            )
            + 1j * w * C1
        ) ** (-1)

    def _P_P_impedance(self, frequency: float = None) -> complex:
        """Function computing the equivalent impedance for a P-P topology WPT system.

        Args:
            frequency (float): Frequency at wich the impedance must be computed. Defaults to None.

        Returns:
            complex: Equivalent impedance of the system viewed by the source at the frequency past in argument.
        """

        R1 = self.transmitter.R
        L1 = self.transmitter.L
        C1 = self.transmitter.C_p
        M = self.M
        R2 = self.reciever.R
        L2 = self.reciever.L
        C2 = self.reciever.C_p
        R_l = self.reciever.R_l
        w = 2 * np.pi * frequency

        return (
            1
            / (
                R1
                + 1j * w * L1
                + (w**2 * M**2) / (R2 + 1j * w * L2 + R_l / (1 + 1j * w * R_l * C2))
            )
            + 1j * w * C1
        ) ** (-1)

    def _SP_S_impedance(self, frequency: float = None) -> complex:
        """Function computing the equivalent impedance for a SP-S topology WPT system.

        Args:
            frequency (float): Frequency at wich the impedance must be computed. Defaults to None.

        Returns:
            complex: Equivalent impedance of the system viewed by the source at the frequency past in argument.
        """

        R1 = self.transmitter.R
        L1 = self.transmitter.L
        C1s = self.transmitter.C_s
        C1p = self.transmitter.C_p
        M = self.M
        R2 = self.reciever.R
        L2 = self.reciever.L
        C2 = self.reciever.C_s
        R_l = self.reciever.R_l
        w = 2 * np.pi * frequency

        return 1 / (1j * w * C1s) + (
            1
            / (
                R1
                + 1j * w * L1
                + (w**2 * M**2) / (R2 + R_l + 1j * w * L2 + 1 / (1j * w * C2))
            )
            + 1j * w * C1p
        ) ** (-1)

    def _SP_P_impedance(self, frequency: float = None) -> complex:
        """Function computing the equivalent impedance for a SP-P topology WPT system.

        Args:
            frequency (float): Frequency at wich the impedance must be computed. Defaults to None.

        Returns:
            complex: Equivalent impedance of the system viewed by the source at the frequency past in argument.
        """

        R1 = self.transmitter.R
        L1 = self.transmitter.L
        C1s = self.transmitter.C_s
        C1p = self.transmitter.C_p
        M = self.M
        R2 = self.reciever.R
        L2 = self.reciever.L
        C2 = self.reciever.C_p
        R_l = self.reciever.R_l
        w = 2 * np.pi * frequency

        return 1 / (1j * w * C1s) + (
            1
            / (
                R1
                + 1j * w * L1
                + (w**2 * M**2) / (R2 + 1j * w * L2 + R_l / (1 + 1j * w * R_l * C2))
            )
            + 1j * w * C1p
        ) ** (-1)

    def _LCC_LCC_impedance(self, frequency: float = None) -> complex:
        """Function computing the equivalent impedance for a LCC-LCC topology WPT system.

        Args:
            frequency (float): Frequency at wich the impedance must be computed. Defaults to None.

        Returns:
            complex: Equivalent impedance of the system viewed by the source at the frequency past in argument.
        """

        R1 = self.transmitter.R
        L1 = self.transmitter.L_s
        L_tx = self.transmitter.L
        C1s = self.transmitter.C_s
        C1p = self.transmitter.C_p
        M = self.M
        R2 = self.reciever.R
        L2 = self.reciever.L_s
        L_rx = self.reciever.L
        C2p = self.reciever.C_p
        C2s = self.reciever.C_s
        R_l = self.reciever.R_l
        w = 2 * np.pi * frequency
        # print(self.name, self.topology)
        # print(f"R1 : {R1}, L1 : {L1}, L_tx : {L_tx}, C1s : {C1s}, C1p : {C1p}, M : {M}, R2 {R2}, L2 : {L2}, L_rx {L_rx}, C2p : {C2p}, C2s : {C2s}, R_l : {R_l} ")
        Z2 = (
            R2
            + 1 / (1j * w * C2s)
            + (L2 * 1j * w + R_l) / (L2 * C2p * (1j * w) ** 2 + R_l * C2p * 1j * w)
        )
        Ztx = (
            R1
            + 1 / (1j * w * C1s)
            + (L_tx - M) * 1j * w
            + (M * (L_rx - M) * (1j * w) ** 2 + M * 1j * w * Z2) / (1j * w * L_rx + Z2)
        )

        return L1 * 1j * w + Ztx / (1 + C1p * 1j * w * Ztx)

    def _LCC_S_impedance(self, frequency: float = None) -> complex:
        """Function computing the equivalent impedance for a LCC-S topology WPT system.

        Args:
            frequency (float): Frequency at wich the impedance must be computed. Defaults to None.

        Returns:
            complex: Equivalent impedance of the system viewed by the source at the frequency past in argument.
        """

        R1 = self.transmitter.R
        L1 = self.transmitter.L_s
        L_tx = self.transmitter.L
        C1s = self.transmitter.C_s
        C1p = self.transmitter.C_p
        M = self.M
        R2 = self.reciever.R
        L_rx = self.reciever.L
        C2s = self.reciever.C_s
        R_l = self.reciever.R_l
        w = 2 * np.pi * frequency

        Z2 = R2 + C2s + R_l
        Ztx = (
            R1
            + 1 / (1j * w * C1s)
            + (L_tx - M) * 1j * w
            + (M * (L_rx - M) * (1j * w) ** 2 + M * 1j * w * Z2) / (1j * w * L_rx + Z2)
        )

        return L1 * 1j * w + Ztx / (1 + C1p * 1j * w * Ztx)

    def _mutual_from_geometry(self):
        """Compute the mutual inductance from the coils geometry.

        Returns:
            float: Mutual inductance between the two coils calulated from the coils geometry.
            The formula can be found at the end of "Optimal Design of ICPT Systems Applied to
            Electric Vehicle Battery Charge" IEEE, 2009.
            The coils are considered to be aligned and centred.
        """

        e = (self.transmitter.height - self.transmitter.height) / 2
        c = (self.transmitter.width - self.transmitter.width) / 2
        d = self.transmitter.width - self.reciever.width - c
        m = self.reciever.width + c
        q = self.reciever.height + e
        g = self.transmitter.width - c
        p = self.transmitter.height - e
        t = self.transmitter.height - self.reciever.height - e
        µ0 = 4 * np.pi * 1e-7
        N1 = self.transmitter.N
        N2 = self.reciever.N
        h = self.h

        return (
            µ0
            / (4 * np.pi)
            * N1
            * N2
            * (
                (
                    d
                    * np.log(
                        (d + (h**2 + (-t) ** 2 + d**2) ** (0.5))
                        / (d + (h**2 + d**2 + (-t) ** 2) ** (0.5))
                    )
                    + h
                    * np.log(
                        (g + (h**2 + q**2 + g**2) ** (0.5))
                        / (g + (h**2 + g**2 + (-t) ** 2) ** (0.5))
                    )
                    + c
                    * np.log(
                        ((-c) + (h**2 + q**2 + c**2) ** (0.5))
                        / ((-c) + (h**2 + c**2 + (-t) ** 2) ** (0.5))
                    )
                    + m
                    * np.log(
                        ((-m) + (h**2 + (-t) ** 2 + m**2) ** (0.5))
                        / ((-m) + (h**2 + m**2 + q**2) ** (0.5))
                    )
                    + (h**2 + q**2 + d**2) ** (0.5)
                    - (h**2 + q**2 + g**2) ** (0.5)
                    - (h**2 + q**2 + m**2) ** (0.5)
                    + (h**2 + q**2 + c**2) ** (0.5)
                    + (h**2 + (-t) ** 2 + g**2) ** (0.5)
                    - (h**2 + (-t) ** 2 + d**2) ** (0.5)
                    + (h**2 + (-t) ** 2 + m**2) ** (0.5)
                    - (h**2 + (-t) ** 2 + c**2) ** (0.5)
                )
                - (
                    d
                    * np.log(
                        (d + (h**2 + (-p) ** 2 + d**2) ** (0.5))
                        / (d + (h**2 + d**2 + e**2) ** (0.5))
                    )
                    + g
                    * np.log(
                        (h + (h**2 + e**2 + g**2) ** (0.5))
                        / (g + (h**2 + h**2 + (-p) ** 2) ** (0.5))
                    )
                    + c
                    * np.log(
                        ((-c) + (h**2 + e**2 + c**2) ** (0.5))
                        / ((-c) + (h**2 + c**2 + (-p) ** 2) ** (0.5))
                    )
                    + m
                    * np.log(
                        ((-m) + (h**2 + (-p) ** 2 + m**2) ** (0.5))
                        / ((-m) + (h**2 + m**2 + e**2) ** (0.5))
                    )
                    + (h**2 + e**2 + d**2) ** (0.5)
                    - (h**2 + e**2 + g**2) ** (0.5)
                    - (h**2 + e**2 + m**2) ** (0.5)
                    + (h**2 + e**2 + c**2) ** (0.5)
                    + (h**2 + (-p) ** 2 + g**2) ** (0.5)
                    - (h**2 + (-p) ** 2 + d**2) ** (0.5)
                    + (h**2 + (-p) ** 2 + m**2) ** (0.5)
                    - (h**2 + (-p) ** 2 + c**2) ** (0.5)
                )
                + (
                    t
                    * np.log(
                        (t + (h**2 + (-g) ** 2 + t**2) ** (0.5))
                        / (t + (h**2 + t**2 + c**2) ** (0.5))
                    )
                    + p
                    * np.log(
                        (p + (h**2 + p**2 + c**2) ** (0.5))
                        / (p + (h**2 + (-g) ** 2 + p**2) ** (0.5))
                    )
                    + e
                    * np.log(
                        ((-e) + (h**2 + e**2 + c**2) ** (0.5))
                        / ((-e) + (h**2 + e**2 + (-g) ** 2) ** (0.5))
                    )
                    + q
                    * np.log(
                        ((-q) + (h**2 + (-g) ** 2 + q**2) ** (0.5))
                        / ((-q) + (h**2 + c**2 + q**2) ** (0.5))
                    )
                    + (h**2 + c**2 + t**2) ** (0.5)
                    - (h**2 + c**2 + p**2) ** (0.5)
                    - (h**2 + c**2 + q**2) ** (0.5)
                    + (h**2 + e**2 + c**2) ** (0.5)
                    + (h**2 + (-g) ** 2 + p**2) ** (0.5)
                    - (h**2 + (-g) ** 2 + t**2) ** (0.5)
                    + (h**2 + (-g) ** 2 + q**2) ** (0.5)
                    - (h**2 + (-g) ** 2 + e**2) ** (0.5)
                )
                - (
                    t
                    * np.log(
                        (t + (h**2 + (-d) ** 2 + t**2) ** (0.5))
                        / (t + (h**2 + t**2 + m**2) ** (0.5))
                    )
                    + p
                    * np.log(
                        (p + (h**2 + m**2 + p**2) ** (0.5))
                        / (p + (h**2 + (-d) ** 2 + p**2) ** (0.5))
                    )
                    + e
                    * np.log(
                        ((-e) + (h**2 + e**2 + m**2) ** (0.5))
                        / ((-e) + (h**2 + e**2 + (-d) ** 2) ** (0.5))
                    )
                    + q
                    * np.log(
                        ((-q) + (h**2 + (-d) ** 2 + q**2) ** (0.5))
                        / ((-q) + (h**2 + m**2 + q**2) ** (0.5))
                    )
                    + (h**2 + m**2 + t**2) ** (0.5)
                    - (h**2 + m**2 + p**2) ** (0.5)
                    - (h**2 + m**2 + q**2) ** (0.5)
                    + (h**2 + e**2 + m**2) ** (0.5)
                    + (h**2 + (-d) ** 2 + p**2) ** (0.5)
                    - (h**2 + (-d) ** 2 + t**2) ** (0.5)
                    + (h**2 + (-d) ** 2 + q**2) ** (0.5)
                    - (h**2 + (-d) ** 2 + e**2) ** (0.5)
                )
            )
        )
