package us.ihmc.euclid.geometry;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class Pose3DTest
{
   private static final double EPSILON = 1e-7;

   @Test
   public void testConstructors()
   {
      Random random = new Random(52942L);
      double x, y, z;
      Quaternion quaternion;
      Pose3D allDoubles, fromComponents, copiedPose, fromRBT;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         allDoubles = new Pose3D(x, y, z, quaternion.getYaw(), quaternion.getPitch(), quaternion.getRoll());

         assertEqualsDelta(x, allDoubles.getX(), EPSILON);
         assertEqualsDelta(y, allDoubles.getY(), EPSILON);
         assertEqualsDelta(z, allDoubles.getZ(), EPSILON);
         assertEqualsDelta(quaternion.getYaw(), allDoubles.getYaw(), EPSILON);
         assertEqualsDelta(quaternion.getPitch(), allDoubles.getPitch(), EPSILON);
         assertEqualsDelta(quaternion.getRoll(), allDoubles.getRoll(), EPSILON);

         fromComponents = new Pose3D(new Point3D(x, y, z), quaternion);

         assertEqualsDelta(x, fromComponents.getX(), EPSILON);
         assertEqualsDelta(y, fromComponents.getY(), EPSILON);
         assertEqualsDelta(z, fromComponents.getZ(), EPSILON);
         assertEqualsDelta(quaternion.getYaw(), fromComponents.getYaw(), EPSILON);
         assertEqualsDelta(quaternion.getPitch(), fromComponents.getPitch(), EPSILON);
         assertEqualsDelta(quaternion.getRoll(), fromComponents.getRoll(), EPSILON);

         copiedPose = new Pose3D(fromComponents);

         assertEqualsDelta(x, fromComponents.getX(), EPSILON);
         assertEqualsDelta(y, fromComponents.getY(), EPSILON);
         assertEqualsDelta(z, fromComponents.getZ(), EPSILON);
         assertEqualsDelta(fromComponents.getYaw(), copiedPose.getYaw(), EPSILON);
         assertEqualsDelta(fromComponents.getPitch(), copiedPose.getPitch(), EPSILON);
         assertEqualsDelta(fromComponents.getRoll(), copiedPose.getRoll(), EPSILON);

         fromRBT = new Pose3D(new RigidBodyTransform(new QuaternionBasedTransform(quaternion, new Point3D(x, y, z))));

         assertEqualsDelta(x, fromRBT.getX(), EPSILON);
         assertEqualsDelta(y, fromRBT.getY(), EPSILON);
         assertEqualsDelta(z, fromRBT.getZ(), EPSILON);
         assertEqualsDelta(quaternion.getYaw(), fromRBT.getYaw(), EPSILON);
         assertEqualsDelta(quaternion.getPitch(), fromRBT.getPitch(), EPSILON);
         assertEqualsDelta(quaternion.getRoll(), fromRBT.getRoll(), EPSILON);
      }
   }

   @Test
   public void testSetToNaN()
   {
      Random random = new Random(70324L);
      Pose3D toSet;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         toSet = EuclidGeometryRandomTools.nextPose3D(random);

         toSet.setToNaN();

         assertEqualsDelta(Double.NaN, toSet.getX(), EPSILON);
         assertEqualsDelta(Double.NaN, toSet.getY(), EPSILON);
         assertEqualsDelta(Double.NaN, toSet.getZ(), EPSILON);
         assertEqualsDelta(Double.NaN, toSet.getYaw(), EPSILON);
         assertEqualsDelta(Double.NaN, toSet.getPitch(), EPSILON);
         assertEqualsDelta(Double.NaN, toSet.getRoll(), EPSILON);
      }
   }

   @Test
   public void testSetToZero()
   {
      Random random = new Random(70924L);
      Pose3D toSet;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         toSet = EuclidGeometryRandomTools.nextPose3D(random);

         toSet.setToZero();

         assertEqualsDelta(0, toSet.getX(), EPSILON);
         assertEqualsDelta(0, toSet.getY(), EPSILON);
         assertEqualsDelta(0, toSet.getZ(), EPSILON);
         assertEqualsDelta(0, toSet.getYaw(), EPSILON);
         assertEqualsDelta(0, toSet.getPitch(), EPSILON);
         assertEqualsDelta(0, toSet.getRoll(), EPSILON);
      }
   }

   @Test
   public void testSetComponents()
   {
      Random random = new Random(71484L);
      double x, y, z;
      Vector3D tuple;
      Quaternion quaternion;
      RotationMatrix rot;
      AxisAngle aa;
      Pose3D toSet, toCopy;

      for (int i = 0; i < ITERATIONS; ++i)
      {
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();

         toSet = EuclidGeometryRandomTools.nextPose3D(random);
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         toSet.setX(x);
         toSet.setY(y);
         toSet.setZ(z);
         toSet.setOrientation(quaternion);

         assertEqualsDelta(x, toSet.getX(), EPSILON);
         assertEqualsDelta(y, toSet.getY(), EPSILON);
         assertEqualsDelta(z, toSet.getZ(), EPSILON);
         assertEqualsDelta(quaternion.getYaw(), toSet.getYaw(), EPSILON);
         assertEqualsDelta(quaternion.getPitch(), toSet.getPitch(), EPSILON);
         assertEqualsDelta(quaternion.getRoll(), toSet.getRoll(), EPSILON);

         toSet = EuclidGeometryRandomTools.nextPose3D(random);

         aa = EuclidCoreRandomTools.nextAxisAngle(random);

         toSet.setPosition(x, y, z);
         toSet.setOrientation(aa);

         assertEqualsDelta(x, toSet.getX(), EPSILON);
         assertEqualsDelta(y, toSet.getY(), EPSILON);
         assertEqualsDelta(z, toSet.getZ(), EPSILON);
         assertEqualsDelta(aa.getYaw(), toSet.getYaw(), EPSILON);
         assertEqualsDelta(aa.getPitch(), toSet.getPitch(), EPSILON);
         assertEqualsDelta(aa.getRoll(), toSet.getRoll(), EPSILON);

         toSet = EuclidGeometryRandomTools.nextPose3D(random);
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         toSet.set(x, y, z, quaternion.getYaw(), quaternion.getPitch(), quaternion.getRoll());

         assertEqualsDelta(x, toSet.getX(), EPSILON);
         assertEqualsDelta(y, toSet.getY(), EPSILON);
         assertEqualsDelta(z, toSet.getZ(), EPSILON);
         assertEqualsDelta(quaternion.getYaw(), toSet.getYaw(), EPSILON);
         assertEqualsDelta(quaternion.getPitch(), toSet.getPitch(), EPSILON);
         assertEqualsDelta(quaternion.getRoll(), toSet.getRoll(), EPSILON);

         toSet = EuclidGeometryRandomTools.nextPose3D(random);

         tuple = new Vector3D(x, y, z);
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);
         toSet.set(tuple, quaternion);

         assertEqualsDelta(x, toSet.getX(), EPSILON);
         assertEqualsDelta(y, toSet.getY(), EPSILON);
         assertEqualsDelta(z, toSet.getZ(), EPSILON);
         assertEqualsDelta(quaternion.getYaw(), toSet.getYaw(), EPSILON);
         assertEqualsDelta(quaternion.getPitch(), toSet.getPitch(), EPSILON);
         assertEqualsDelta(quaternion.getRoll(), toSet.getRoll(), EPSILON);

         toSet = EuclidGeometryRandomTools.nextPose3D(random);
         rot = EuclidCoreRandomTools.nextRotationMatrix(random);

         toSet.setPosition(tuple);
         toSet.setOrientation(rot);

         assertEqualsDelta(x, toSet.getX(), EPSILON);
         assertEqualsDelta(y, toSet.getY(), EPSILON);
         assertEqualsDelta(z, toSet.getZ(), EPSILON);
         assertEqualsDelta(rot.getYaw(), toSet.getYaw(), EPSILON);
         assertEqualsDelta(rot.getPitch(), toSet.getPitch(), EPSILON);
         assertEqualsDelta(rot.getRoll(), toSet.getRoll(), EPSILON);

         toSet = EuclidGeometryRandomTools.nextPose3D(random);

         double[] ypr = {random.nextDouble(), random.nextDouble(), random.nextDouble()};
         toCopy = new Pose3D();
         toCopy.setPosition(x, y, z);
         toCopy.setOrientationYawPitchRoll(ypr);

         assertEqualsDelta(ypr[0], toCopy.getYaw(), EPSILON);
         assertEqualsDelta(ypr[1], toCopy.getPitch(), EPSILON);
         assertEqualsDelta(ypr[2], toCopy.getRoll(), EPSILON);

         toCopy.setOrientationYawPitchRoll(ypr[0], ypr[1], ypr[2]);

         assertEqualsDelta(ypr[0], toCopy.getYaw(), EPSILON);
         assertEqualsDelta(ypr[1], toCopy.getPitch(), EPSILON);
         assertEqualsDelta(ypr[2], toCopy.getRoll(), EPSILON);

         toSet.set(toCopy);

         assertEqualsDelta(x, toSet.getX(), EPSILON);
         assertEqualsDelta(y, toSet.getY(), EPSILON);
         assertEqualsDelta(z, toSet.getZ(), EPSILON);
         assertEqualsDelta(ypr[0], toSet.getYaw(), EPSILON);
         assertEqualsDelta(ypr[1], toSet.getPitch(), EPSILON);
         assertEqualsDelta(ypr[2], toSet.getRoll(), EPSILON);
      }
   }

   @Test
   public void testPointDistance()
   {
      Random random = new Random(41133L);
      Pose3D firstPose, secondPose;
      Point3D point = new Point3D();
      Vector3D translation;
      double length;

      for (int i = 0; i < ITERATIONS; i++)
      {
         firstPose = EuclidGeometryRandomTools.nextPose3D(random);
         secondPose = new Pose3D(firstPose);

         translation = EuclidCoreRandomTools.nextVector3D(random);
         length = translation.length();

         secondPose.appendTranslation(translation);

         assertEqualsDelta(length, firstPose.getPositionDistance(secondPose), EPSILON);

         point.set(firstPose.getPosition());
         point.add(translation);

         assertEqualsDelta(length, firstPose.getPositionDistance(point), EPSILON);
      }
   }

   // TODO
   /*
    * @Test public void testOrientationDistance() { }
    */

   @Test
   public void testEquals()
   {
      Random random = new Random(9827L);
      Pose3D firstPose, secondPose;
      double x = random.nextDouble() - random.nextDouble();
      double y = random.nextDouble() - random.nextDouble();
      double z = random.nextDouble() - random.nextDouble();
      Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);
      Vector3D translation, orthogonal, normal = new Vector3D();

      firstPose = new Pose3D(new Point3D(x, y, z), quaternion);
      secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

      // Sanity checks
      assertTrue(firstPose.equals(secondPose));
      assertTrue(secondPose.equals(secondPose));
      assertTrue(firstPose.equals(firstPose));
      assertTrue(secondPose.equals(secondPose));

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal if and only if point components are exactly equal
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         firstPose = new Pose3D(new Point3D(x, y, z), quaternion);
         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         translation = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EPSILON);
         secondPose.appendTranslation(translation);

         assertFalse(firstPose.equals(secondPose));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal if and only if orientation angles are exactly equal
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         firstPose = new Pose3D(new Point3D(x, y, z), quaternion);

         quaternion.getRotationVector(normal);
         orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normal, true);
         normal.applyTransform(new RigidBodyTransform(new AxisAngle(orthogonal, EPSILON), new Vector3D()));

         secondPose = new Pose3D(x, y, z, normal.angle(new Vector3D(0, 0, 1)), normal.angle(new Vector3D(0, 1, 0)), normal.angle(new Vector3D(1, 0, 0)));

         assertFalse(firstPose.equals(secondPose));
      }
   }

   @Test
   public void testEpsilonEquals()
   {
      Random random = new Random(9827L);
      Pose3D firstPose, secondPose;
      double x = random.nextDouble() - random.nextDouble();
      double y = random.nextDouble() - random.nextDouble();
      double z = random.nextDouble() - random.nextDouble();
      double qx, qy, qz, qs;
      Quaternion quaternion = EuclidCoreRandomTools.nextQuaternion(random);

      firstPose = new Pose3D(new Point3D(x, y, z), quaternion);
      secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

      // Sanity checks
      assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));
      assertTrue(secondPose.epsilonEquals(firstPose, EPSILON));
      assertTrue(firstPose.epsilonEquals(firstPose, EPSILON));
      assertTrue(secondPose.epsilonEquals(secondPose, EPSILON));

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal when distance between point components is <= epsilon
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         firstPose = new Pose3D(new Point3D(x, y, z), quaternion);
         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         secondPose.setX(x + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         secondPose.setY(y + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         secondPose.setZ(z + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         secondPose.setX(x + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         secondPose.setY(y + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         secondPose.setZ(z + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Poses are equal when difference between components of quaternions are each <= epsilon
         x = random.nextDouble() - random.nextDouble();
         y = random.nextDouble() - random.nextDouble();
         z = random.nextDouble() - random.nextDouble();
         quaternion = EuclidCoreRandomTools.nextQuaternion(random);

         qx = quaternion.getX();
         qy = quaternion.getY();
         qz = quaternion.getZ();
         qs = quaternion.getS();

         firstPose = new Pose3D(new Point3D(x, y, z), quaternion);

         quaternion.setUnsafe(qx + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON, qy, qz, qs);

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON, qz, qs);

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy, qz + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON, qs);

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy, qz, qs + (random.nextBoolean() ? 1 : -1) * 0.99 * EPSILON);

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         assertTrue(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON, qy, qz, qs);

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON, qz, qs);

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy, qz + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON, qs);

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));

         quaternion.setUnsafe(qx, qy, qz, qs + (random.nextBoolean() ? 1 : -1) * 1.01 * EPSILON);

         secondPose = new Pose3D(new Point3D(x, y, z), quaternion);

         assertFalse(firstPose.epsilonEquals(secondPose, EPSILON));
      }
   }
}
