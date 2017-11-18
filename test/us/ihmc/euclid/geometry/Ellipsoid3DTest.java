package us.ihmc.euclid.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class Ellipsoid3DTest
{
   private static final double epsilon = 1e-4;    // This epsilon is meant small changes in coordinates. Use Ellipsoid3d's DEFAULT_EPSILON for error handling.
   private static final int iterations = 100;

	
	@Test
   public void testCommonShape3dFunctionality()
   {
      Shape3DTestHelper testHelper = new Shape3DTestHelper();
      Random random = new Random(1776L);

      int numberOfShapes = 1000;
      int numberOfPoints = 1000;

      for (int i = 0; i < numberOfShapes; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         double xRadius = EuclidCoreRandomTools.generateRandomDouble(random, 0.02, 10.0);
         double yRadius = EuclidCoreRandomTools.generateRandomDouble(random, 0.02, 10.0);
         double zRadius = EuclidCoreRandomTools.generateRandomDouble(random, 0.02, 10.0);
         Ellipsoid3D ellipsoid3d = new Ellipsoid3D(transform, xRadius, yRadius, zRadius);
         testHelper.runSimpleTests(ellipsoid3d, random, numberOfPoints);
      }
   }

	
	@Test
   public void testSimpleWithNoTransform()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      Ellipsoid3D ellipsoid = new Ellipsoid3D(xRadius, yRadius, zRadius);
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xRadius - epsilon, 0.0, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yRadius - epsilon, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, zRadius - epsilon)));

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xRadius + epsilon, 0.0, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -yRadius + epsilon, 0.0)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, -zRadius + epsilon)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xRadius + epsilon, 0.0, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yRadius + epsilon, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, zRadius + epsilon)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-xRadius - epsilon, 0.0, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -yRadius - epsilon, 0.0)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, -zRadius - epsilon)));
      
      Point3D closestPointToPack = new Point3D();
      Vector3D normalToPack = new Vector3D();
      boolean isInside = ellipsoid.checkIfInside(new Point3D(xRadius - epsilon, 0.0, 0.0), closestPointToPack, normalToPack);
      assertTrue(isInside);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(1.0, 0.0, 0.0), normalToPack, 1e-7);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(xRadius, 0.0, 0.0), closestPointToPack, 1e-7);
      
      isInside = ellipsoid.checkIfInside(new Point3D(3.0, 7.0, -12.0), closestPointToPack, normalToPack);
      assertFalse(isInside);
      double xScaled = closestPointToPack.getX()/xRadius;
      double yScaled = closestPointToPack.getY()/yRadius;
      double zScaled = closestPointToPack.getZ()/zRadius;
      double sumSquared = xScaled*xScaled + yScaled*yScaled + zScaled*zScaled;
      assertEquals(1.0, sumSquared, 1e-7);
   }

	
	@Test
   public void testSimpleWithTranslationalTransform()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      double xTranslation = 0.5;
      double yTranslation = -0.3;
      double zTranslation = 1.1;
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslation(new Vector3D(xTranslation, yTranslation, zTranslation));

      Ellipsoid3D ellipsoid = new Ellipsoid3D(transform, xRadius, yRadius, zRadius);
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation + xRadius - epsilon, yTranslation, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation + yRadius - epsilon, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation + zRadius - epsilon)));

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation-xRadius + epsilon, yTranslation, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation -yRadius + epsilon, zTranslation)));
      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation-zRadius + epsilon)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation + xRadius + epsilon, yTranslation, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation + yRadius + epsilon, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation + zRadius + epsilon)));

      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation-xRadius - epsilon, yTranslation, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation-yRadius - epsilon, zTranslation)));
      assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation-zRadius - epsilon)));
   }

	
	@Test
   public void testExampleUsage()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;
      
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(Math.PI / 2.0);
      transform.setTranslation(new Vector3D(0.0, 5.0, 0.0));

      Ellipsoid3D ellipsoid = new Ellipsoid3D(transform, xRadius, yRadius, zRadius);

      assertTrue(ellipsoid.isInsideEpsilon(new Point3D(0.0, 8.0, 0.0), 0.001));
      assertTrue(ellipsoid.isInsideEpsilon(new Point3D(0.0, 7.9, 0.0), 0.001));
      assertFalse(ellipsoid.isInsideEpsilon(new Point3D(0.0, 8.2, 0.0), 0.001));
      assertFalse(ellipsoid.isInsideEpsilon(new Point3D(0.2, 8.2, 0.2), 0.001));
   }

	
	@Test
   public void testSet()
   {
      double xRadius = 1.0;
      double yRadius = 2.0;
      double zRadius = 3.0;

      Ellipsoid3D ellipsoid = new Ellipsoid3D(xRadius, yRadius, zRadius);

      assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));

      double[] zeroes = new double[] {0.0, 0.0, 0.0};

      double[] radii = new double[] {xRadius, yRadius, zRadius};

      assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, zeroes, zeroes, ellipsoid);
      assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, zeroes, new double[] {-epsilon, -epsilon, -epsilon}, ellipsoid);
      assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(true, radii, zeroes, new double[] {epsilon, epsilon, epsilon}, ellipsoid);
   }

   private void assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(boolean invertResult, double[] radii, double[] translations, double[] offsets,
           Ellipsoid3D ellipsoid)
   {
      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3D(translations[0] + radii[0] + offsets[0], translations[1], translations[2])));
      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3D(translations[0] - (radii[0] + offsets[0]), translations[1], translations[2])));

      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3D(translations[0], translations[1] + radii[1] + offsets[1], translations[2])));
      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3D(translations[0], translations[1] - (radii[1] + offsets[1]), translations[2])));

      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3D(translations[0], translations[1], translations[2] + radii[2] + offsets[2])));
      assertEquals(!invertResult, ellipsoid.isInsideOrOnSurface(new Point3D(translations[0], translations[1], translations[2] - (radii[2] + offsets[2]))));
   }

	
	@Test
   public void testTranslation()
   {
      for (int i = 0; i < iterations; i++)
      {
         Random random = new Random(1776L);

         double xRadius = random.nextDouble();
         double yRadius = random.nextDouble();
         double zRadius = random.nextDouble();

         double[] radii = new double[] {xRadius, yRadius, zRadius};

         double xTranslation = random.nextDouble();
         double yTranslation = random.nextDouble();
         double zTranslation = random.nextDouble();

         double[] translations = new double[] {xTranslation, yTranslation, zTranslation};

         double[] zeroes = new double[] {0.0, 0.0, 0.0};

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setTranslation(new Vector3D(xTranslation, yTranslation, zTranslation));

         Ellipsoid3D ellipsoid = new Ellipsoid3D(xRadius, yRadius, zRadius);
         ellipsoid.setPose(transform);

         assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xTranslation, yTranslation, zTranslation)));

         assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, translations, zeroes, ellipsoid);
         assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(false, radii, translations, new double[] {-epsilon, -epsilon, -epsilon}, ellipsoid);
         assertPointIsInsideOrOnSurfaceForPlusOrMinusXYAndZ(true, radii, translations, new double[] {epsilon, epsilon, epsilon}, ellipsoid);
      }
   }

	
	@Test
   public void testSimpleRotations()
   {
      for (int n = 0; n < iterations; n++)
      {
         Random random = new Random(1776L);

         double xRadius = random.nextDouble();
         double yRadius = random.nextDouble();
         double zRadius = random.nextDouble();

         double[] angles = new double[] {Math.PI / 3.0, Math.PI / 4.0, Math.PI / 6.0};

         for (double angle : angles)
         {
            for (int i = 0; i < 3; i++)
            {
               RigidBodyTransform transform = new RigidBodyTransform();

               if (i == 0)
                  transform.setRotationRollAndZeroTranslation(angle);
               if (i == 1)
                  transform.setRotationPitchAndZeroTranslation(angle);
               if (i == 2)
                  transform.setRotationYawAndZeroTranslation(angle);

               Ellipsoid3D ellipsoid = new Ellipsoid3D(xRadius, yRadius, zRadius);
               ellipsoid.setPose(transform);

               assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, 0.0)));

               if (i == 0)
               {
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xRadius, 0.0, 0.0)));

                  double yCoord = yRadius * Math.cos(angle);
                  double zCoord = yRadius * Math.sin(angle);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yCoord, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -yCoord, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yCoord + epsilon, zCoord + epsilon)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -(yCoord + epsilon), -(zCoord + epsilon))));

                  yCoord = zRadius * Math.cos(angle + Math.PI / 2.0);
                  zCoord = zRadius * Math.sin(angle + Math.PI / 2.0);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yCoord, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -yCoord, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yCoord - epsilon, zCoord + epsilon)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, -(yCoord - epsilon), -(zCoord + epsilon))));
               }

               if (i == 1)
               {
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, yRadius, 0.0)));

                  double xCoord = xRadius * Math.cos(angle);
                  double zCoord = -xRadius * Math.sin(angle);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord, 0.0, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xCoord, 0.0, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord + epsilon, 0.0, zCoord - epsilon)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-(xCoord + epsilon), 0.0, -(zCoord - epsilon))));

                  xCoord = zRadius * Math.cos(angle + Math.PI / 2.0);
                  zCoord = -zRadius * Math.sin(angle + Math.PI / 2.0);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord, 0.0, zCoord)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xCoord, 0.0, -zCoord)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord - epsilon, 0.0, zCoord - epsilon)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-(xCoord - epsilon), 0.0, -(zCoord - epsilon))));
               }

               if (i == 2)
               {
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(0.0, 0.0, zRadius)));

                  double xCoord = xRadius * Math.cos(angle);
                  double yCoord = xRadius * Math.sin(angle);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord, yCoord, 0.0)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xCoord, -yCoord, 0.0)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord + epsilon, yCoord + epsilon, 0.0)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-(xCoord + epsilon), -(yCoord + epsilon), 0.0)));

                  xCoord = yRadius * Math.cos(angle + Math.PI / 2.0);
                  yCoord = yRadius * Math.sin(angle + Math.PI / 2.0);

                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord, yCoord, 0.0)));
                  assertTrue(ellipsoid.isInsideOrOnSurface(new Point3D(-xCoord, -yCoord, 0.0)));

                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(xCoord - epsilon, yCoord + epsilon, 0.0)));
                  assertFalse(ellipsoid.isInsideOrOnSurface(new Point3D(-(xCoord - epsilon), -(yCoord + epsilon), 0.0)));
               }
            }
         }
      }
   }

	
	@Test
   public void testMethodsForRandomEllipsoids()
   {
      Random random = new Random(1865L);
      Ellipsoid3D ellipsoid = new Ellipsoid3D(1.0, 2.0, 3.0);
      double xRad, yRad, zRad;
      RigidBodyTransform transform = randomTransform(random);

      for (int n = 0; n < iterations; n++)
      {
         xRad = random.nextDouble();
         yRad = random.nextDouble();
         zRad = random.nextDouble();

         ellipsoid.setRadiusX(xRad);
         ellipsoid.setRadiusY(yRad);
         ellipsoid.setRadiusZ(zRad);
         ellipsoid.setPose(transform);

         Ellipsoid3D ellipsoidCopy = new Ellipsoid3D(ellipsoid);

         assertEquals(ellipsoid.getRadiusX(), ellipsoidCopy.getRadiusX(), 1e-10);
         assertEquals(ellipsoid.getRadiusY(), ellipsoidCopy.getRadiusY(), 1e-10);
         assertEquals(ellipsoid.getRadiusZ(), ellipsoidCopy.getRadiusZ(), 1e-10);

         Point3D center = new Point3D();
         ellipsoid.getPosition(center);

         Point3D centerCopy = new Point3D();
         ellipsoidCopy.getPosition(centerCopy);

         EuclidCoreTestTools.assertTuple3DEquals(center, centerCopy, 1e-10);
      }
   }

   @Test
   public void testGeometricallyEquals() {
      Random random = new Random(89725L);
      Ellipsoid3D firstEllipsoid, secondEllipsoid;
      double radiusX, radiusY, radiusZ;
      double epsilon = 1e-7;
      Vector3D translation;

      radiusX = random.nextDouble();
      radiusY = random.nextDouble();
      radiusZ = random.nextDouble();

      firstEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
      secondEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);

      assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));
      assertTrue(secondEllipsoid.geometricallyEquals(firstEllipsoid, epsilon));
      assertTrue(firstEllipsoid.geometricallyEquals(firstEllipsoid, epsilon));
      assertTrue(secondEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

      for (int i = 0; i < iterations; ++i) {
         radiusX = random.nextDouble();
         radiusY = random.nextDouble();
         radiusZ = random.nextDouble();

         firstEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);

         // SecondBox = (LX, HZ, WY)
         secondEllipsoid = new Ellipsoid3D(radiusX, radiusZ, radiusY);

         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

         secondEllipsoid.appendTransform(new RigidBodyTransform(new AxisAngle(1.0, 0.0, 0.0, Math.PI / 2.0), new Vector3D()));

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

         // SecondBox = (WY, LX, HZ)
         secondEllipsoid = new Ellipsoid3D(radiusY, radiusX, radiusZ);

         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

         secondEllipsoid.appendTransform(new RigidBodyTransform(new AxisAngle(0.0, 0.0, 1.0, Math.PI / 2.0), new Vector3D()));

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

         // SecondBox = (HZ, WY, LX)
         secondEllipsoid = new Ellipsoid3D(radiusZ, radiusY, radiusX);

         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

         secondEllipsoid.appendTransform(new RigidBodyTransform(new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0), new Vector3D()));

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

         // SecondBox = (HZ, LX, WY)
         secondEllipsoid = new Ellipsoid3D(radiusZ, radiusX, radiusY);

         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

         secondEllipsoid.appendTransform(new RigidBodyTransform(0,1,0,0,0,0,1,0,1,0,0,0));

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

         // SecondBox = (WY, HZ, LX)
         secondEllipsoid = new Ellipsoid3D(radiusY, radiusZ, radiusX);

         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));

         secondEllipsoid.appendTransform(new RigidBodyTransform(0,0,1,0,1,0,0,0,0,1,0,0));

         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));
      }
      
      for (int i = 0; i < iterations; ++i) {
         radiusX = random.nextDouble();
         radiusY = random.nextDouble();
         radiusZ = random.nextDouble();

         firstEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
         
         secondEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
         translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 0.99 * epsilon);
         secondEllipsoid.appendTranslation(translation);
         
         assertTrue(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));
         
         secondEllipsoid = new Ellipsoid3D(radiusX, radiusY, radiusZ);
         translation = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.01 * epsilon);
         secondEllipsoid.appendTranslation(translation);
         
         assertFalse(firstEllipsoid.geometricallyEquals(secondEllipsoid, epsilon));
      }
   }

   private RigidBodyTransform randomTransform(Random random)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      RigidBodyTransform tempTransform = new RigidBodyTransform();
      transform.setRotationRollAndZeroTranslation(2 * Math.PI * random.nextDouble());
      tempTransform.setRotationPitchAndZeroTranslation(2 * Math.PI * random.nextDouble());
      transform.multiply(tempTransform);
      tempTransform.setRotationYawAndZeroTranslation(2 * Math.PI * random.nextDouble());
      transform.multiply(tempTransform);

      double[] matrix = new double[16];
      transform.get(matrix);
      matrix[3] = random.nextDouble();
      matrix[7] = random.nextDouble();
      matrix[11] = random.nextDouble();

      return transform;
   }
}
