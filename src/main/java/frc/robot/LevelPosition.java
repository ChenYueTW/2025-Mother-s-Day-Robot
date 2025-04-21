package frc.robot;

public class LevelPosition {
    public enum ElevatorLevel {
        L1(0), L2(0.8599853515625),
        L3(2.380126953125), L4(4.70147705078125),
        High(3.44384765625), Low(2.07391357421875),
        Source(0.0), Processer(0.01);

        private final double angle;

        ElevatorLevel(double angle) {
            this.angle = angle;
        }

        public double get() {
            return this.angle;
        }
    }

    public enum LifterLevel {
        L1(0), L2(0.032225485804851156),
        L3(0.032225485804851156), L4(0.022745185019294376),
        High(0.4649761835033076), Low(0.4908814687844542),
        Source(0.0), Processer(0.4308880150909592);

        private final double angle;

        LifterLevel(double angle) {
            this.angle = angle;
        }

        public double get() {
            return this.angle;
        }
    }
}
