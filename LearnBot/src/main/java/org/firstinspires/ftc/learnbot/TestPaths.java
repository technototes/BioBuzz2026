package org.firstinspires.ftc.learnbot;

import static com.pedropathing.api.Paths.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Line;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.paths.interpolator.PiecewiseInterpolator;

/**** DO NOT EDIT ****
 These paths are specifically for testing the visualizer. If you want to make some
 changes to the 'real' paths, just create a different file...
 **** DO NOT EDIT ****/

@Configurable
public class TestPaths {

    private static final PoseFactory poseFactory = PoseFactory.degrees();

    public static double org = 15.0;
    public static double edge = 50.0;
    public static double orgu = 130.0;
    public static double edgeu = 90.0;
    public static double extra = 25.0;
    public static double extra2 = 27.0;
    public static double one80 = 180;
    public static double refVal = edge;
    public static int sixty = 60;
    public static int ninetyD = 90;
    public static double ninety = ninetyD;

    public static final Pose start = poseFactory.of(org, org, 0);
    public static final Pose step1 = poseFactory.of(edge, org, ninety);
    public static final Pose step2 = poseFactory.of(edge, refVal, 35);
    public static final Pose step3 = poseFactory.of(extra, extra2, sixty);
    public static final Pose step4 = poseFactory.of(orgu, orgu, one80);
    public static final Pose startu = poseFactory.of(org, org, 0);
    public static final Pose step1u = poseFactory.of(edge, org, ninety);
    public static final Pose step2u = poseFactory.of(edge, refVal, 35);
    public static final Pose step3u = poseFactory.of(extra, extra2, sixty);
    public static final Pose step4u = poseFactory.of(org, org, one80);
    public static final Pose stepb = poseFactory.of(extra, extra, sixty);
    public static final Pose stepc = poseFactory.of(15, 20, 0);
    public static final Pose stepd = poseFactory.of(18, 55, 135);

    public Path start_to_step1() {
        return line(start, step1).linear(start, step1);
    }

    public Path unused1() {
        return curve(step1, step2, step4, step1);
    }

    public Path u1_u2() {
        return line(step1, poseFactory.of(org, edge, 0));
    }

    public Path unused2() {
        return line(poseFactory.of(org, edge, 0), start);
    }

    public Path u2_u3() {
        return line(start, poseFactory.of(edge, 5, 15));
    }

    public Path unused3() {
        return curve(poseFactory.of(edge, 5, 15), start, poseFactory.of(5, 5, 0));
    }

    public Path u3_u4() {
        return line(poseFactory.of(5, 5, 0), start);
    }

    public Path unused4() {
        return curve(
            start,
            poseFactory.of(15, 25, 0),
            poseFactory.of(55, 44, 0),
            poseFactory.of(10, org, 0),
            poseFactory.of(edge, 10, Math.toRadians(sixty)),
            step1
        );
    }

    public Path u4_ol() {
        return line(step1, stepc);
    }

    public Path otherLine() {
        return line(stepc, stepd);
    }

    public Path start_to_step1_5() {
        return line(startu, step1u);
    }

    public Path unused1_5() {
        return curve(step1u, step2u, step4u, step1u);
    }

    public Path u1_u2_5() {
        return line(step1u, poseFactory.of(orgu, edgeu, 0));
    }

    public Path unused2_5() {
        return line(poseFactory.of(orgu, edgeu, 0), startu);
    }

    public Path u2_u3_5() {
        return line(startu, poseFactory.of(edgeu, 95, 15));
    }

    public Path unused3_5() {
        return curve(poseFactory.of(edgeu, 95, 15), startu, poseFactory.of(95, 95, 0));
    }

    public Path u3_u4_5() {
        return line(poseFactory.of(5, 5, 0), startu);
    }

    public Path unused4_5() {
        return curve(
            startu,
            poseFactory.of(95, 125, 0),
            poseFactory.of(85, 133, 0),
            poseFactory.of(130, orgu, 0),
            poseFactory.of(edgeu, 10, Math.toRadians(sixty)),
            step1u
        );
    }

    public Path u4_ol_5() {
        return line(step1, stepc);
    }

    public Path otherLine_5() {
        return line(stepc, stepd);
    }

    public Pose getStart() {
        return start;
    }

    public Path Path1() {
        return path(
            start_to_step1(),
            unused1(),
            curve(step1, poseFactory.of(10, extra, 0), step4, poseFactory.of(edge, 10, 0), step1)
        ).linear(0, Math.toRadians(90));
    }

    public Path Path2() {
        return curve(step1, stepb, step2).constant(step3);
    }

    public Path Path3() {
        return line(step2, step3).tangent();
    }

    public Path Path4() {
        return curve(step3, step1u, step4).heading(
            Interpolator.piecewise()
                .until(0.2, Interpolator.tangent)
                .until(0.4, Interpolator.facingPoint(Vector2D.cartesian(5, 5)))
                .until(0.6, Interpolator.constant(Math.toRadians(90)))
                .until(0.8, Interpolator.linear(Math.toRadians(90), Math.PI))
                .until(1, Interpolator.longLinear(Math.PI, Math.toRadians(90)))
        );
    }

    public Path Path5() {
        return path(
            unused1_5(),
            u1_u2_5(),
            unused2_5(),
            u2_u3_5(),
            unused3_5(),
            u3_u4_5(),
            unused4_5(),
            u4_ol_5(),
            otherLine_5()
        ).facingPoint(Vector2D.cartesian(1, 1));
    }
}
