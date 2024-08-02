linux-4.1.15

Code based on:
https://github.com/melexis/mlx90640-library


Hardware:
正点原子ATK-DL6Y2C

The display function is not implemented yet.

device tree:

    &i2c1 {
        clock-frequency = <100000>;
        pinctrl-names = "default";
        pinctrl-0 = <&pinctrl_i2c1>;
        status = "okay";

        mlx90640@33 {
            compatible = "drivertest,mlx90640";
            reg = <0x33>;
        };
    };