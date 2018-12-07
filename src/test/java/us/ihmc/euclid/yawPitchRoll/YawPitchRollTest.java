package us.ihmc.euclid.yawPitchRoll;

import static us.ihmc.robotics.Assert.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Quaternion;

public class YawPitchRollTest extends YawPitchRollBasicsTest<YawPitchRoll>
{
   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(423534);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll ypr = new YawPitchRoll();
         assertTrue(ypr.getYaw() == 0.0);
         assertTrue(ypr.getPitch() == 0.0);
         assertTrue(ypr.getRoll() == 0.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Quaternion q = EuclidCoreRandomTools.nextQuaternion(random);
         YawPitchRoll ypr = new YawPitchRoll(q);
         assertEquals(ypr.getYaw(), q.getYaw(), getEpsilon());
         assertEquals(ypr.getPitch(), q.getPitch(), getEpsilon());
         assertEquals(ypr.getRoll(), q.getRoll(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double yaw = random.nextDouble();
         double pitch = random.nextDouble();
         double roll = random.nextDouble();
         YawPitchRoll ypr = new YawPitchRoll(yaw, pitch, roll);
         assertTrue(ypr.getYaw() == yaw);
         assertTrue(ypr.getPitch() == pitch);
         assertTrue(ypr.getRoll() == roll);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         double[] yprArray = {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()};
         YawPitchRoll ypr = new YawPitchRoll(yprArray);
         assertTrue(ypr.getYaw() == yprArray[0]);
         assertTrue(ypr.getPitch() == yprArray[1]);
         assertTrue(ypr.getRoll() == yprArray[2]);
      }
   }

   @Test
   public void testSet()
   {
      super.testSet();

      Random random = new Random(356);

      for (int i = 0; i < ITERATIONS; i++)
      {
         YawPitchRoll ypr1 = EuclidCoreRandomTools.nextYawPitchRoll(random);
         YawPitchRoll ypr2 = new YawPitchRoll();

         assertFalse(ypr1.epsilonEquals(ypr2, getEpsilon()));
         ypr2.set(ypr1);
         assertTrue(ypr1.epsilonEquals(ypr2, getEpsilon()));
         EuclidCoreTestTools.assertYawPitchRollEquals(ypr1, ypr2, getEpsilon());
      }
   }

   @Test
   public void testHashCode() throws Exception
   {
      Random random = new Random(621541L);
      YawPitchRoll ypr = new YawPitchRoll();

      ypr.setYaw(random.nextDouble());
      ypr.setPitch(random.nextDouble());
      ypr.setRoll(random.nextDouble());

      int newHashCode, previousHashCode;
      newHashCode = ypr.hashCode();
      assertEquals(newHashCode, ypr.hashCode());

      previousHashCode = ypr.hashCode();

      for (int i = 0; i < ITERATIONS; i++)
      {
         ypr.setElement(i % 3, random.nextDouble());
         newHashCode = ypr.hashCode();
         assertNotEquals(newHashCode, previousHashCode);
         previousHashCode = newHashCode;
      }
   }

   @Override
   public YawPitchRoll createEmptyYawPitchRoll()
   {
      return new YawPitchRoll();
   }

   @Override
   public YawPitchRoll createRandomYawPitchRoll(Random random)
   {
      return EuclidCoreRandomTools.nextYawPitchRoll(random);
   }

   @Override
   public YawPitchRoll createYawPitchRoll(double yaw, double pitch, double roll)
   {
      return new YawPitchRoll(yaw, pitch, roll);
   }

   @Override
   public YawPitchRoll createYawPitchRoll(Orientation3DReadOnly orientation3D)
   {
      return new YawPitchRoll(orientation3D);
   }

   @Override
   public double getEpsilon()
   {
      return 1.0e-13;
   }

   @Override
   public double getSmallestEpsilon()
   {
      return 1.0e-15;
   }
}
