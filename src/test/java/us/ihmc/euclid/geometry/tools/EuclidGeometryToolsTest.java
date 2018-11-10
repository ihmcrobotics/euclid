package us.ihmc.euclid.geometry.tools;

import static org.junit.Assert.*;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public class EuclidGeometryToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;
   private static final double LARGE_EPSILON = 2.0e-9;

   @Test
   public void testAngleFromFirstToSecondVector2D() throws Exception
   {
      Random random = new Random(4353L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D firstVector = EuclidCoreRandomTools.nextVector2D(random);
         Vector2D secondVector = EuclidCoreRandomTools.nextVector2D(random);

         firstVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         secondVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double expectedAngle = firstVector.angle(secondVector);
         double actualAngle = EuclidGeometryTools.angleFromFirstToSecondVector2D(firstVector.getX(), firstVector.getY(), secondVector.getX(),
                                                                                 secondVector.getY());

         EuclidCoreTestTools.assertAngleEquals(expectedAngle, actualAngle, EPSILON);
      }
   }

   @Test
   public void testAngleFromXForwardToVector2D() throws Exception
   {
      Random random = new Random(4353L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D xForward = new Vector2D(1.0, 0.0);
         Vector2D vector = EuclidCoreRandomTools.nextVector2D(random);
         vector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double expectedAngle = xForward.angle(vector);
         double actualAngle = EuclidGeometryTools.angleFromXForwardToVector2D(vector);
         EuclidCoreTestTools.assertAngleEquals(expectedAngle, actualAngle, EPSILON);

         actualAngle = EuclidGeometryTools.angleFromXForwardToVector2D(vector.getX(), vector.getY());
         EuclidCoreTestTools.assertAngleEquals(expectedAngle, actualAngle, EPSILON);
      }
   }

   @Test
   public void testAngleFromFirstToSecondVector3D() throws Exception
   {
      Random random = new Random(4353L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D secondVector = EuclidCoreRandomTools.nextVector3D(random);

         firstVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         secondVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double expectedAngle = firstVector.angle(secondVector);
         double actualAngle = EuclidGeometryTools.angleFromFirstToSecondVector3D(firstVector.getX(), firstVector.getY(), firstVector.getZ(),
                                                                                 secondVector.getX(), secondVector.getY(), secondVector.getZ());

         EuclidCoreTestTools.assertAngleEquals(expectedAngle, actualAngle, EPSILON);
      }
   }

   @Test
   public void testAreLine2DsCollinear() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D lineDirection1 = EuclidCoreRandomTools.nextVector2D(random);
         lineDirection1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double angleEpsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);

         Vector2D lineDirection2 = new Vector2D();
         RotationMatrixTools.applyYawRotation(rotationAngle, lineDirection1, lineDirection2);
         lineDirection2.normalize();
         lineDirection2.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         Point2D firstPointOnLine1 = EuclidCoreRandomTools.nextPoint2D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D secondPointOnLine1 = new Point2D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, firstPointOnLine1);

         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection1);
         orthogonal.normalize();
         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double distanceEspilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Point2D firstPointOnLine2 = new Point2D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point2D secondPointOnLine2 = new Point2D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);

         boolean expectedCollinear = rotationAngle < angleEpsilon && distance < distanceEspilon;
         boolean actualCollinear = EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2,
                                                                           angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);

         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine1, lineDirection1, firstPointOnLine2, lineDirection2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);

         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine1, lineDirection1, firstPointOnLine2, secondPointOnLine2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
      }

      // Test only the distance with parallel line segments.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2D(random);
         lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         orthogonal.normalize();

         double angleEpsilon = EuclidGeometryTools.ONE_MILLIONTH;

         Point2D firstPointOnLine1 = EuclidCoreRandomTools.nextPoint2D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D secondPointOnLine1 = new Point2D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine1);

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double distanceEspilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Point2D firstPointOnLine2 = new Point2D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point2D secondPointOnLine2 = new Point2D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine2);
         firstPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine2);

         boolean expectedCollinear = distance < distanceEspilon;
         boolean actualCollinear;
         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine1, secondPointOnLine1, secondPointOnLine2, firstPointOnLine2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(secondPointOnLine1, firstPointOnLine1, secondPointOnLine2, firstPointOnLine2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(secondPointOnLine1, firstPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);

         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine2, secondPointOnLine2, firstPointOnLine1, secondPointOnLine1, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(secondPointOnLine2, firstPointOnLine2, firstPointOnLine1, secondPointOnLine1, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(secondPointOnLine2, firstPointOnLine2, secondPointOnLine1, firstPointOnLine1, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine2, secondPointOnLine2, secondPointOnLine1, firstPointOnLine1, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
      }
   }

   @Test
   public void testAreLine3DsCollinear() throws Exception
   {
      Random random = new Random(2312L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D lineDirection1 = EuclidCoreRandomTools.nextRotationVector(random);
         lineDirection1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double angleEpsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);
         Vector3D orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineDirection1, true);
         AxisAngle axisAngle = new AxisAngle(orthogonal, rotationAngle);

         Vector3D lineDirection2 = new Vector3D();
         axisAngle.transform(lineDirection1, lineDirection2);
         lineDirection2.normalize();
         lineDirection2.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         Point3D firstPointOnLine1 = EuclidCoreRandomTools.nextPoint3D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D secondPointOnLine1 = new Point3D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection1, firstPointOnLine1);

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double distanceEspilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Point3D firstPointOnLine2 = new Point3D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point3D secondPointOnLine2 = new Point3D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, firstPointOnLine2);

         boolean expectedCollinear = rotationAngle < angleEpsilon && distance < distanceEspilon;
         boolean actualCollinear = EuclidGeometryTools.areLine3DsCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2,
                                                                           angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine3DsCollinear(firstPointOnLine1, lineDirection1, firstPointOnLine2, lineDirection2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
      }

      // Test only the distance with parallel line segments.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D lineDirection = EuclidCoreRandomTools.nextVector3D(random);
         lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineDirection, true);

         double angleEpsilon = EuclidGeometryTools.ONE_MILLIONTH;

         Point3D firstPointOnLine1 = EuclidCoreRandomTools.nextPoint3D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D secondPointOnLine1 = new Point3D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine1);

         double distance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double distanceEspilon = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Point3D firstPointOnLine2 = new Point3D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point3D secondPointOnLine2 = new Point3D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine2);
         firstPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine2);

         boolean expectedCollinear = distance < distanceEspilon;
         boolean actualCollinear;
         actualCollinear = EuclidGeometryTools.areLine3DsCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine3DsCollinear(firstPointOnLine1, secondPointOnLine1, secondPointOnLine2, firstPointOnLine2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine3DsCollinear(secondPointOnLine1, firstPointOnLine1, secondPointOnLine2, firstPointOnLine2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine3DsCollinear(secondPointOnLine1, firstPointOnLine1, firstPointOnLine2, secondPointOnLine2, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);

         actualCollinear = EuclidGeometryTools.areLine3DsCollinear(firstPointOnLine2, secondPointOnLine2, firstPointOnLine1, secondPointOnLine1, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine3DsCollinear(secondPointOnLine2, firstPointOnLine2, firstPointOnLine1, secondPointOnLine1, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine3DsCollinear(secondPointOnLine2, firstPointOnLine2, secondPointOnLine1, firstPointOnLine1, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
         actualCollinear = EuclidGeometryTools.areLine3DsCollinear(firstPointOnLine2, secondPointOnLine2, secondPointOnLine1, firstPointOnLine1, angleEpsilon,
                                                                   distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
      }
   }

   @Test
   public void testArePlane3DsCoincident() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane1 = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal1 = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         Point3D pointOnPlane2 = new Point3D();
         Vector3D planeNormal2 = new Vector3D();

         double distanceEpsilon = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double distanceBetweenPlanes = EuclidCoreRandomTools.nextDouble(random, 1.0);

         pointOnPlane2.scaleAdd(distanceBetweenPlanes, planeNormal1, pointOnPlane1);

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal1, true);
         double angleEpsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);

         AxisAngle rotationAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         rotationMatrix.transform(planeNormal1, planeNormal2);

         boolean expectedCoincidentResult = Math.abs(distanceBetweenPlanes) < distanceEpsilon && rotationAngle < angleEpsilon;
         boolean actualCoincidentResult = EuclidGeometryTools.arePlane3DsCoincident(pointOnPlane1, planeNormal1, pointOnPlane2, planeNormal2, angleEpsilon,
                                                                                    distanceEpsilon);
         assertEquals(expectedCoincidentResult, actualCoincidentResult);
      }
   }

   @Test
   public void testAreVector2DsParallel() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D firstVector = EuclidCoreRandomTools.nextVector2D(random);
         firstVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double angleEpsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);

         Vector2D secondVector = new Vector2D();
         RotationMatrixTools.applyYawRotation(rotationAngle, firstVector, secondVector);
         secondVector.normalize();
         secondVector.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         assertEquals(rotationAngle < angleEpsilon, EuclidGeometryTools.areVector2DsParallel(firstVector, secondVector, angleEpsilon));
      }

      // Try again with small values
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D firstVector = EuclidCoreRandomTools.nextVector2D(random);
         firstVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double angleEpsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, EuclidGeometryTools.ONE_MILLIONTH * Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 0.0, EuclidGeometryTools.ONE_MILLIONTH * Math.PI / 2.0);
         if (Math.abs(rotationAngle - angleEpsilon) < 1.0e-7)
            continue; // This is the limit of accuracy.

         Vector2D secondVector = new Vector2D();
         RotationMatrixTools.applyYawRotation(rotationAngle, firstVector, secondVector);

         assertEquals(rotationAngle < angleEpsilon, EuclidGeometryTools.areVector2DsParallel(firstVector, secondVector, angleEpsilon));
      }

      Vector2D unitVector = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
      Vector2D firstVector = new Vector2D(unitVector);
      Vector2D secondVector = new Vector2D(unitVector);
      firstVector.scale(0.9 * EuclidGeometryTools.ONE_TEN_MILLIONTH);

      assertFalse(EuclidGeometryTools.areVector2DsParallel(firstVector, secondVector, Math.PI / 2.0));
      assertFalse(EuclidGeometryTools.areVector2DsParallel(secondVector, firstVector, Math.PI / 2.0));

      // Check that with an epsilon exactly equal to zero it does not crash.
      EuclidGeometryTools.areVector2DsParallel(firstVector, secondVector, 0.0);

      try
      {
         EuclidGeometryTools.areVector2DsParallel(firstVector, secondVector, -Double.MIN_VALUE);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         EuclidGeometryTools.areVector2DsParallel(firstVector, secondVector, Math.PI / 2.0 + 1.110224e-16);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testAreVector3DsParallel() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.nextVector3D(random);
         firstVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, firstVector, true);
         double angleEpsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);

         AxisAngle rotationAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.normalize();
         secondVector.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         assertEquals(rotationAngle < angleEpsilon, EuclidGeometryTools.areVector3DsParallel(firstVector, secondVector, angleEpsilon));
      }

      // Try again with small values
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.nextVector3D(random);
         firstVector.normalize();

         Vector3D rotationAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, firstVector, true);
         double angleEpsilon = EuclidCoreRandomTools.nextDouble(random, 0.0, EuclidGeometryTools.ONE_MILLIONTH * Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 0.0, EuclidGeometryTools.ONE_MILLIONTH * Math.PI / 2.0);
         if (Math.abs(rotationAngle - angleEpsilon) < 1.0e-7)
            continue; // This is the limit of accuracy.

         AxisAngle rotationAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);

         assertEquals(rotationAngle < angleEpsilon, EuclidGeometryTools.areVector3DsParallel(firstVector, secondVector, angleEpsilon));
      }

      Vector3D unitVector = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
      Vector3D firstVector = new Vector3D(unitVector);
      Vector3D secondVector = new Vector3D(unitVector);
      firstVector.scale(0.9 * EuclidGeometryTools.ONE_TEN_MILLIONTH);

      assertFalse(EuclidGeometryTools.areVector3DsParallel(firstVector, secondVector, Math.PI / 2.0));
      assertFalse(EuclidGeometryTools.areVector3DsParallel(secondVector, firstVector, Math.PI / 2.0));

      // Check that with an epsilon exactly equal to zero it does not crash.
      EuclidGeometryTools.areVector3DsParallel(firstVector, secondVector, 0.0);

      try
      {
         EuclidGeometryTools.areVector3DsParallel(firstVector, secondVector, -Double.MIN_VALUE);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         EuclidGeometryTools.areVector3DsParallel(firstVector, secondVector, Math.PI / 2.0 + 1.110224e-16);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testAveragePoint2Ds() throws Exception
   {
      assertNull(EuclidGeometryTools.averagePoint2Ds(new ArrayList<>()));

      ArrayList<Point2D> points = new ArrayList<>();
      Point2D a = new Point2D(1.0, 4.6);
      Point2D b = new Point2D(5.2, 6.0);
      Point2D c = new Point2D(3.7, 2.0);
      points.add(a);
      points.add(b);
      points.add(c);
      double expectedReturn1 = 3.3;
      double expectedReturn2 = 4.2;
      Point2D actualReturn = EuclidGeometryTools.averagePoint2Ds(points);
      double actualReturn1 = actualReturn.getX();
      double actualReturn2 = actualReturn.getY();
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      ArrayList<Point2D> points1 = new ArrayList<>();
      Point2D a1 = new Point2D(0.0, 0.0);
      Point2D b1 = new Point2D(0.0, 0.0);
      Point2D c1 = new Point2D(0.0, 0.0);
      points1.add(a1);
      points1.add(b1);
      points1.add(c1);
      double expectedReturn11 = 0.0;
      double expectedReturn12 = 0.0;
      Point2D actualReturn01 = EuclidGeometryTools.averagePoint2Ds(points1);
      double actualReturn11 = actualReturn01.getX();
      double actualReturn12 = actualReturn01.getY();
      assertEquals("return value", expectedReturn11, actualReturn11, EPSILON);
      assertEquals("return value", expectedReturn12, actualReturn12, EPSILON);

      ArrayList<Point2D> points2 = new ArrayList<>();
      Point2D a2 = new Point2D(-1.0, -4.6);
      Point2D b2 = new Point2D(-5.2, -6.0);
      Point2D c2 = new Point2D(-3.7, -2.0);
      points2.add(a2);
      points2.add(b2);
      points2.add(c2);
      double expectedReturn21 = -3.3;
      double expectedReturn22 = -4.2;
      Point2D actualReturn02 = EuclidGeometryTools.averagePoint2Ds(points2);
      double actualReturn21 = actualReturn02.getX();
      double actualReturn22 = actualReturn02.getY();
      assertEquals("return value", expectedReturn21, actualReturn21, EPSILON);
      assertEquals("return value", expectedReturn22, actualReturn22, EPSILON);
   }

   @Test
   public void testAveragePoint3Ds() throws Exception
   {
      assertNull(EuclidGeometryTools.averagePoint3Ds(new ArrayList<>()));

      ArrayList<Point3D> points = new ArrayList<>();
      Point3D a = new Point3D(4.3, 5.6, 3.6);
      Point3D b = new Point3D(8.1, 8.4, 0.0);
      Point3D c = new Point3D(5.6, 1.0, 4.5);
      points.add(a);
      points.add(b);
      points.add(c);
      double expectedReturn1 = 6.0;
      double expectedReturn2 = 5.0;
      double expectedReturn3 = 2.7;
      Point3D actualReturn = EuclidGeometryTools.averagePoint3Ds(points);
      double actualReturn1 = actualReturn.getX();
      double actualReturn2 = actualReturn.getY();
      double actualReturn3 = actualReturn.getZ();
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);
      assertEquals("return value", expectedReturn3, actualReturn3, EPSILON);

      ArrayList<Point3D> points1 = new ArrayList<>();
      Point3D a1 = new Point3D(0.0, 0.0, 0.0);
      Point3D b1 = new Point3D(0.0, 0.0, 0.0);
      Point3D c1 = new Point3D(0.0, 0.0, 0.0);
      points1.add(a1);
      points1.add(b1);
      points1.add(c1);
      double expectedReturn11 = 0.0;
      double expectedReturn12 = 0.0;
      double expectedReturn13 = 0.0;
      Point3D actualReturn01 = EuclidGeometryTools.averagePoint3Ds(points1);
      double actualReturn11 = actualReturn01.getX();
      double actualReturn12 = actualReturn01.getY();
      double actualReturn13 = actualReturn01.getZ();
      assertEquals("return value", expectedReturn11, actualReturn11, EPSILON);
      assertEquals("return value", expectedReturn12, actualReturn12, EPSILON);
      assertEquals("return value", expectedReturn13, actualReturn13, EPSILON);
   }

   @Test
   public void testAverageTwoPoint3Ds() throws Exception
   {

      Point3D a = new Point3D(5.8, 9.9, 4.5);
      Point3D b = new Point3D(5.6, 8.1, 5.5);
      double expectedReturn1 = 5.7;
      double expectedReturn2 = 9.0;
      double expectedReturn3 = 5;
      Point3D actualReturn = EuclidGeometryTools.averagePoint3Ds(a, b);
      double actualReturn1 = actualReturn.getX();
      double actualReturn2 = actualReturn.getY();
      double actualReturn3 = actualReturn.getZ();
      assertEquals("return value", expectedReturn1, actualReturn1, EPSILON);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);
      assertEquals("return value", expectedReturn3, actualReturn3, EPSILON);

      Point3D a1 = new Point3D(-5, -5, -5);
      Point3D b1 = new Point3D(-5, -5, -5);
      double expectedReturn11 = -5;
      double expectedReturn12 = -5;
      double expectedReturn13 = -5;
      Point3D actualReturn01 = EuclidGeometryTools.averagePoint3Ds(a1, b1);
      double actualReturn11 = actualReturn01.getX();
      double actualReturn12 = actualReturn01.getY();
      double actualReturn13 = actualReturn01.getZ();
      assertEquals("return value", expectedReturn11, actualReturn11, EPSILON);
      assertEquals("return value", expectedReturn12, actualReturn12, EPSILON);
      assertEquals("return value", expectedReturn13, actualReturn13, EPSILON);

      Point3D a2 = new Point3D(0, 0, 0);
      Point3D b2 = new Point3D(0, 0, 0);
      double expectedReturn21 = 0;
      double expectedReturn22 = 0;
      double expectedReturn23 = 0;
      Point3D actualReturn02 = EuclidGeometryTools.averagePoint3Ds(a2, b2);
      double actualReturn21 = actualReturn02.getX();
      double actualReturn22 = actualReturn02.getY();
      double actualReturn23 = actualReturn02.getZ();
      assertEquals("return value", expectedReturn21, actualReturn21, EPSILON);
      assertEquals("return value", expectedReturn22, actualReturn22, EPSILON);
      assertEquals("return value", expectedReturn23, actualReturn23, EPSILON);
   }

   @Test
   public void testAxisAngleFromFirstToSecondVector3D() throws Exception
   {
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.nextVector3D(random);
         firstVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI);
         Vector3D expectedAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         Vector3D secondVector = new Vector3D();
         expectedAxisAngle.transform(firstVector, secondVector);
         secondVector.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(firstVector, secondVector, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), EuclidGeometryTools.ONE_TRILLIONTH);
         assertEquals(0.0, actualAxis.dot(firstVector), EuclidGeometryTools.ONE_TRILLIONTH);
         assertEquals(0.0, actualAxis.dot(secondVector), EuclidGeometryTools.ONE_TRILLIONTH);

         assertEquals(0.0, expectedAxis.dot(firstVector), EuclidGeometryTools.ONE_TRILLIONTH);
         assertEquals(0.0, expectedAxis.dot(secondVector), EuclidGeometryTools.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EuclidGeometryTools.ONE_TRILLIONTH);
      }

      // Test close to 0.0
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.nextVector3D(random);
         firstVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, 0.0001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         Vector3D expectedAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         Vector3D secondVector = new Vector3D();
         expectedAxisAngle.transform(firstVector, secondVector);
         secondVector.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(firstVector, secondVector, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), EuclidGeometryTools.ONE_TRILLIONTH);
         // Can not be as accurate as we get closer to 0.0
         assertEquals(0.0, actualAxis.dot(firstVector), 1.0e-10);
         assertEquals(0.0, actualAxis.dot(secondVector), 1.0e-10);

         assertEquals(0.0, expectedAxis.dot(firstVector), EuclidGeometryTools.ONE_TRILLIONTH);
         assertEquals(0.0, expectedAxis.dot(secondVector), EuclidGeometryTools.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         // Can not be as accurate as we get closer to 0.0
         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, 1.0e-10);
      }

      // Test close to Math.PI
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.nextVector3D(random);
         firstVector.scale(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         double expectedAngle = EuclidCoreRandomTools.nextDouble(random, 0.00001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         expectedAngle += Math.PI;
         Vector3D expectedAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         Vector3D secondVector = new Vector3D();
         expectedAxisAngle.transform(firstVector, secondVector);
         secondVector.scale(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(firstVector, secondVector, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(0.0, expectedAxis.dot(firstVector), EuclidGeometryTools.ONE_TRILLIONTH);
         assertEquals(0.0, expectedAxis.dot(secondVector), EuclidGeometryTools.ONE_TRILLIONTH);

         assertEquals(1.0, actualAxis.length(), EuclidGeometryTools.ONE_TRILLIONTH);
         // Can not be as accurate as we get closer to Math.PI
         assertEquals(0.0, actualAxis.dot(firstVector), 1.0e-10);
         assertEquals(0.0, actualAxis.dot(secondVector), 1.0e-10);
         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         // Can not be as accurate as we get closer to pi
         EuclidCoreTestTools.assertAxisAngleGeometricallyEquals(expectedAxisAngle, actualAxisAngle, 1.0e-10);
      }

      // Test exactly at 0.0
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D secondVector = new Vector3D(firstVector);
         firstVector.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         secondVector.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         double expectedAngle = 0.0;
         Vector3D expectedAxis = new Vector3D(1.0, 0.0, 0.0);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         AxisAngle actualAxisAngle = new AxisAngle();
         EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(firstVector, secondVector, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), EuclidGeometryTools.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EuclidGeometryTools.ONE_TRILLIONTH);
      }

      // Test exactly at Math.PI
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D referenceNormal = EuclidCoreRandomTools.nextVector3D(random);
         Vector3D rotatedNormal = new Vector3D();
         referenceNormal.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         rotatedNormal.setAndNegate(referenceNormal);
         rotatedNormal.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         double expectedAngle = Math.PI;
         Vector3D expectedAxis = new Vector3D(1.0, 0.0, 0.0);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         AxisAngle actualAxisAngle = new AxisAngle();
         EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), EuclidGeometryTools.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EuclidGeometryTools.ONE_TRILLIONTH);
      }

      // Test axisAngleFromZUpToVector3D(Vector3DReadOnly vector, AxisAngleBasics rotationToPack) & axisAngleFromZUpToVector3D(Vector3DReadOnly vector)
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D zUp = new Vector3D(0.0, 0.0, 1.0);
         Vector3D vector = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);
         AxisAngle expectedAxisAngle = new AxisAngle();
         AxisAngle actualAxisAngle = new AxisAngle();
         EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(zUp, vector, expectedAxisAngle);
         EuclidGeometryTools.axisAngleFromZUpToVector3D(vector, actualAxisAngle);
         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EuclidGeometryTools.ONE_TRILLIONTH);
         actualAxisAngle = EuclidGeometryTools.axisAngleFromZUpToVector3D(vector);
         EuclidCoreTestTools.assertAxisAngleEquals(expectedAxisAngle, actualAxisAngle, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testClosestPoint3DsBetweenTwoLine3Ds() throws Exception
   {
      Point3D expectedPointOnLine1ToPack = new Point3D();
      Point3D expectedPointOnLine2ToPack = new Point3D();

      Point3D actualPointOnLine1ToPack = new Point3D();
      Point3D actualPointOnLine2ToPack = new Point3D();
      Random random = new Random(116L);

      // Most usual case: the lines are not parallel, not intersecting.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.nextVector3D(random);
         lineDirection1.scale(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0));

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);
         lineDirection2.scale(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0));

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         // Rotate line2 around the vector we shifted it along, so it
         // preserves the minimum distance.
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 0.05, Math.PI - 0.05);
         AxisAngle axisAngleAroundShiftVector = new AxisAngle(orthogonalToLine1, rotationAngle);
         axisAngleAroundShiftVector.transform(lineDirection2);

         // At this point, lineStart1 and lineStart2 are expected to be the
         // closest points.
         expectedPointOnLine1ToPack.set(lineStart1);
         expectedPointOnLine2ToPack.set(lineStart2);

         double actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                             actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, 1.0e-10);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, 1.0e-10);

         // Let's shift lineStart1 and lineStart2 along their respective line
         // direction so they're not the closest points.
         lineStart1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                      actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, 1.0e-10);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, 1.0e-10);
      }

      // Test the parallel case. There's an infinite number of solutions but
      // only one minimum distance between the two lines.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         double actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                             actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line
         // direction (the minimum distance should remain the same).
         lineStart1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                      actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Intersection case: the lines are not parallel, but intersecting.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.nextVector3D(random);
         lineDirection1.scale(EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0));

         // Set the intersection point randomly on line1
         Point3D intersection = new Point3D();
         intersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, lineStart1);

         // Set both closest points to the intersection
         expectedPointOnLine1ToPack.set(intersection);
         expectedPointOnLine2ToPack.set(intersection);

         // Create line2 such that it intersects line1 at intersection
         Point3D lineStart2 = new Point3D(intersection);
         Vector3D lineDirection2 = EuclidCoreRandomTools.nextVector3D(random);
         lineDirection2.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         lineStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, lineStart2);

         double actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                             actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(0.0, actualMinimumDistance, 1.0e-10);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, 1.0e-10);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, 1.0e-10);
      }
   }

   @Test
   public void testClosestPoint3DsBetweenTwoLineSegment3Ds() throws Exception
   {
      Point3D expectedPointOnLineSegment1 = new Point3D();
      Point3D expectedPointOnLineSegment2 = new Point3D();

      Point3D actualPointOnLineSegment1 = new Point3D();
      Point3D actualPointOnLineSegment2 = new Point3D();

      Vector3D lineSegmentDirection1 = new Vector3D();
      Vector3D lineSegmentDirection2 = new Vector3D();

      Random random = new Random(1176L);

      // Easy case, the closest points on inside each line segment bounds.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest within bounds of line segment 1
         expectedPointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), orthogonalToLineSegment1, expectedPointOnLineSegment1);

         // Set the line direction 2 to be the rotation of 1 around the shift
         // direction used to create the expectedPointOnLineSegment2
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 0.05, Math.PI - 0.05);
         AxisAngle rotationAroundShiftVector = new AxisAngle(orthogonalToLineSegment1, rotationAngle);
         rotationAroundShiftVector.transform(lineSegmentDirection1, lineSegmentDirection2);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);

         double eps = 1.0e-10;
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, eps);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, eps);
      }

      // Parallel case, expecting expectedPointOnLineSegment1 =
      // lineSegmentStart1
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // expectedPointOnLineSegment1 = lineSegmentStart1
         expectedPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), orthogonalToLineSegment1, expectedPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);

         double eps = 1.0e-10;
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, eps);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, eps);

         // Set the end points of the line segment 2 before the expected
         // closest point, so we have expectedClosestPointOnLineSegment2 =
         // lineSegmentEnd2
         double shiftStartFromExpected = EuclidCoreRandomTools.nextDouble(random, -20.0, -10.0);
         double shiftEndFromExpected = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         lineSegmentStart2.scaleAdd(shiftStartFromExpected, lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(shiftEndFromExpected, lineSegmentDirection2, expectedPointOnLineSegment2);
         expectedPointOnLineSegment2.set(lineSegmentEnd2);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);

         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);

         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);
      }

      // Case: on closest point on lineSegment1 outside end points.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         expectedPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, expectedPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);
      }

      // Edge case: both closest points are outside bounds of each line
      // segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         expectedPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, expectedPointOnLineSegment1);

         // set the start of the second line segment to the expected closest
         // point
         Point3D lineSegmentStart2 = new Point3D(expectedPointOnLineSegment2);

         // Set the line direction 2 to point somewhat in the same direction
         // as the shift vector
         Vector3D orthogonalToShiftVector = EuclidCoreRandomTools.nextOrthogonalVector3D(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, EPSILON);
      }
   }

   @Test
   public void testTriangleArea() throws Exception
   {
      Random random = new Random(1176L);
      // Test for right rectangle, should be half the area of the
      // corresponding rectangle
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = EuclidCoreRandomTools.nextPoint2D(random);
         a.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D b = new Point2D();
         Point2D c = new Point2D();
         Point2D d = new Point2D();

         Vector2D rectangleLength = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);
         Vector2D rectangleWidth = new Vector2D(-rectangleLength.getY(), rectangleLength.getX());
         double length = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double width = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         rectangleLength.scale(length);
         rectangleWidth.scale(width);

         b.add(a, rectangleLength);
         c.add(b, rectangleWidth);
         d.add(a, rectangleWidth);

         double expectedArea = 0.5 * length * width;
         double actualArea = EuclidGeometryTools.triangleArea(a, b, c);
         assertEquals(expectedArea, actualArea, EPSILON);
         actualArea = EuclidGeometryTools.triangleArea(a, c, d);
         assertEquals(expectedArea, actualArea, EPSILON);
         actualArea = EuclidGeometryTools.triangleArea(b, c, d);
         assertEquals(expectedArea, actualArea, EPSILON);
         actualArea = EuclidGeometryTools.triangleArea(a, b, d);
         assertEquals(expectedArea, actualArea, EPSILON);

         // Just an annoying case
         assertEquals(0.0, EuclidGeometryTools.triangleArea(a, a, c), EPSILON);
      }
   }

   @Test
   public void testDistanceBetweenPoint2Ds() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPoint = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D secondPoint = EuclidCoreRandomTools.nextPoint2D(random);

         firstPoint.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         secondPoint.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double expectedDistance = firstPoint.distance(secondPoint);
         double actualDistance = EuclidGeometryTools.distanceBetweenPoint2Ds(firstPoint.getX(), firstPoint.getY(), secondPoint.getX(), secondPoint.getY());
         assertEquals(expectedDistance, actualDistance, EPSILON);

         actualDistance = EuclidGeometryTools.distanceBetweenPoint2Ds(firstPoint.getX(), firstPoint.getY(), secondPoint);
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }
   }

   @Test
   public void testDistanceSquaredBetweenPoint2Ds() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPoint = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D secondPoint = EuclidCoreRandomTools.nextPoint2D(random);

         firstPoint.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         secondPoint.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double expectedDistance = firstPoint.distanceSquared(secondPoint);
         double actualDistance = EuclidGeometryTools.distanceSquaredBetweenPoint2Ds(firstPoint.getX(), firstPoint.getY(), secondPoint.getX(),
                                                                                    secondPoint.getY());
         assertEquals(expectedDistance, actualDistance, EPSILON);

         actualDistance = EuclidGeometryTools.distanceSquaredBetweenPoint2Ds(firstPoint.getX(), firstPoint.getY(), secondPoint);
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }
   }

   @Test
   public void testDistanceBetweenPoint3Ds() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D firstPoint = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D secondPoint = EuclidCoreRandomTools.nextPoint3D(random);

         firstPoint.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         secondPoint.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double expectedDistance = firstPoint.distance(secondPoint);
         double actualDistance = EuclidGeometryTools.distanceBetweenPoint3Ds(firstPoint.getX(), firstPoint.getY(), firstPoint.getZ(), secondPoint.getX(),
                                                                             secondPoint.getY(), secondPoint.getZ());
         assertEquals(expectedDistance, actualDistance, EPSILON);

         actualDistance = EuclidGeometryTools.distanceBetweenPoint3Ds(firstPoint.getX(), firstPoint.getY(), firstPoint.getZ(), secondPoint);
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }
   }

   @Test
   public void testDistanceSquaredBetweenPoint3Ds() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D firstPoint = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D secondPoint = EuclidCoreRandomTools.nextPoint3D(random);

         firstPoint.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         secondPoint.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double expectedDistance = firstPoint.distanceSquared(secondPoint);
         double actualDistance = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(firstPoint.getX(), firstPoint.getY(), firstPoint.getZ(), secondPoint.getX(),
                                                                                    secondPoint.getY(), secondPoint.getZ());
         assertEquals(expectedDistance, actualDistance, EPSILON);

         actualDistance = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(firstPoint.getX(), firstPoint.getY(), firstPoint.getZ(), secondPoint);
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }
   }

   @Test
   public void testDistanceBetweenTwoLine3Ds() throws Exception
   {
      Point3D closestPointOnLine1 = new Point3D();
      Point3D closestPointOnLine2 = new Point3D();

      Random random = new Random(176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.5, 10.0));

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         // Rotate line2 around the vector we shifted it along, so it
         // preserves the minimum distance.
         AxisAngle axisAngleAroundShiftVector = new AxisAngle(orthogonalToLine1, EuclidCoreRandomTools.nextDouble(random, Math.PI));
         RotationMatrix rotationMatrixAroundShiftVector = new RotationMatrix();
         rotationMatrixAroundShiftVector.set(axisAngleAroundShiftVector);
         rotationMatrixAroundShiftVector.transform(lineDirection2);

         // At this point, lineStart1 and lineStart2 are expected to be the
         // closest points.
         closestPointOnLine1.set(lineStart1);
         closestPointOnLine2.set(lineStart2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line
         // direction so they're not the closest points.
         lineStart1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Test the parallel case. There's an infinite number of solutions but
      // only one minimum distance between the two lines.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line
         // direction (the minimum distance should remain the same).
         lineStart1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }
   }

   @Test
   public void testDistanceBetweenTwoLineSegment3Ds() throws Exception
   {
      Point3D closestPointOnLineSegment1 = new Point3D();
      Point3D closestPointOnLineSegment2 = new Point3D();

      Vector3D lineSegmentDirection1 = new Vector3D();
      Vector3D lineSegmentDirection2 = new Vector3D();

      Random random = new Random(11762L);

      // Easy case, the closest points on inside each line segment bounds.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest within bounds of line segment 1
         closestPointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the line direction 2 to be the rotation of 1 around the shift
         // direction used to create the expectedPointOnLineSegment2
         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, 2.0 * Math.PI);
         AxisAngle rotationAroundShiftVector = new AxisAngle(orthogonalToLineSegment1, rotationAngle);
         rotationAroundShiftVector.transform(lineSegmentDirection1, lineSegmentDirection2);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Parallel case, expecting expectedPointOnLineSegment1 =
      // lineSegmentStart1
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // expectedPointOnLineSegment1 = lineSegmentStart1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Set the end points of the line segment 2 before the expected
         // closest point, so we have expectedClosestPointOnLineSegment2 =
         // lineSegmentEnd2
         double shiftStartFromExpected = EuclidCoreRandomTools.nextDouble(random, -20.0, -10.0);
         double shiftEndFromExpected = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         lineSegmentStart2.scaleAdd(shiftStartFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(shiftEndFromExpected, lineSegmentDirection2, closestPointOnLineSegment2);
         closestPointOnLineSegment2.set(lineSegmentEnd2);
         expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Case: on closest point on lineSegment1 outside end points.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);
         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Edge case: both closest points are outside bounds of each line
      // segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         double alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, alpha);
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // set the start of the second line segment to the expected closest
         // point
         Point3D lineSegmentStart2 = new Point3D(closestPointOnLineSegment2);

         // Set the line direction 2 to point somewhat in the same direction
         // as the shift vector
         Vector3D orthogonalToShiftVector = EuclidCoreRandomTools.nextOrthogonalVector3D(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentEnd2 = new Point3D();
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0);
         lineSegmentEnd2.scaleAdd(alpha2, lineSegmentDirection2, closestPointOnLineSegment2);

         double expectedMinimumDistance = closestPointOnLineSegment1.distance(closestPointOnLineSegment2);
         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }
   }

   @Test
   public void testDistanceFromPoint2DToLine2D() throws Exception
   {
      Random random = new Random(243234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine = EuclidCoreRandomTools.nextPoint2D(random);
         pointOnLine.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2D(random);
         lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         orthogonal.normalize();
         if (random.nextBoolean())
            orthogonal.negate();

         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Point2D query = new Point2D();
         query.scaleAdd(expectedDistance, orthogonal, pointOnLine);
         query.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, query);

         double actualDistance = EuclidGeometryTools.distanceFromPoint2DToLine2D(query, pointOnLine, lineDirection);
         assertEquals(expectedDistance, actualDistance, EPSILON);

         // Test with a line direction that a magnitude that is too small
         lineDirection.normalize();
         lineDirection.scale(0.999999 * EuclidGeometryTools.ONE_TRILLIONTH);
         expectedDistance = pointOnLine.distance(query);
         actualDistance = EuclidGeometryTools.distanceFromPoint2DToLine2D(query, pointOnLine, lineDirection);
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }
   }

   @Test
   public void testDistanceFromPoint2DToLineSegment2D() throws Exception
   {
      Point2D point = new Point2D(10, 2);
      Point2D lineStart = new Point2D(4, 2);
      Point2D lineEnd = new Point2D(10, 10);
      double expectedReturn = 4.8;
      double actualReturn = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      Point2D point1 = new Point2D(10, 10);
      Point2D lineStart1 = new Point2D(4, 2);
      Point2D lineEnd1 = new Point2D(4, 2);
      double expectedReturn1 = 10.0;
      double actualReturn1 = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point1, lineStart1, lineEnd1);
      assertEquals("return value", expectedReturn1, actualReturn1, Double.MIN_VALUE);

      Point2D point2 = new Point2D(1, 1);
      Point2D lineStart2 = new Point2D(5, 5);
      Point2D lineEnd2 = new Point2D(5, 5);
      double expectedReturn2 = lineStart2.distance(point2);
      double actualReturn2 = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point2, lineStart2, lineEnd2);
      assertEquals("return value", expectedReturn2, actualReturn2, EPSILON);

      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         EuclidGeometryTools.perpendicularVector2D(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D projection = new Point2D();
         Point2D testPoint = new Point2D();
         double expectedDistance, actualDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentEnd);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testDistanceFromPoint2DToRay2D() throws Exception
   {
      Random random = new Random(43254L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Position the query in front of the ray
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random, -10.0, 10.0);

         Vector2D orthogonalToRay = EuclidGeometryTools.perpendicularVector2D(rayDirection);
         orthogonalToRay.normalize();

         Point2D query = new Point2D();
         // Position the query on the ray in front of the origin
         query.scaleAdd(nextDouble(random, 0.0, 10.0), rayDirection, rayOrigin);
         // Shift it orthogonally
         query.scaleAdd(nextDouble(random, 10.0), orthogonalToRay, query);

         // Distance should be the same as the distance to the line collinear to the ray
         double expectedDistance = EuclidGeometryTools.distanceFromPoint2DToLine2D(query, rayOrigin, rayDirection);
         double actualDistance = EuclidGeometryTools.distanceFromPoint2DToRay2D(query, rayOrigin, rayDirection);
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Position the query behind the ray's origin
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random, -10.0, 10.0);

         Vector2D orthogonalToRay = EuclidGeometryTools.perpendicularVector2D(rayDirection);
         orthogonalToRay.normalize();

         Point2D query = new Point2D();
         // Position the query on the ray behind the origin
         query.scaleAdd(nextDouble(random, -10.0, 0.0), rayDirection, rayOrigin);
         // Shift it orthogonally
         query.scaleAdd(nextDouble(random, 10.0), orthogonalToRay, query);

         // Distance should be the same as the distance to the line collinear to the ray
         double expectedDistance = query.distance(rayOrigin);
         double actualDistance = EuclidGeometryTools.distanceFromPoint2DToRay2D(query, rayOrigin, rayDirection);
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }
   }

   @Test
   public void testDistanceFromPoint3DToLine3D() throws Exception
   {
      Random random = new Random(1176L);
      Point3D point = new Point3D(10, 2, 0);
      Point3D lineStart = new Point3D(4, 2, 0);
      Point3D lineEnd = new Point3D(10, 10, 0);
      double expectedReturn = 4.8;
      double actualReturn = EuclidGeometryTools.distanceFromPoint3DToLine3D(point, lineStart, lineEnd);
      assertEquals("return value", expectedReturn, actualReturn, Double.MIN_VALUE);

      Point3D point1 = new Point3D(10, 10, 0);
      Point3D lineStart1 = new Point3D(4, 2, 0);
      Point3D lineEnd1 = new Point3D(4, 2, 0);
      double expectedReturn1 = 10.0;
      double actualReturn1 = EuclidGeometryTools.distanceFromPoint3DToLine3D(point1, lineStart1, lineEnd1);
      assertEquals("return value", expectedReturn1, actualReturn1, Double.MIN_VALUE);

      for (int i = 0; i < ITERATIONS; i++)
      {
         // Generate a random line
         Point3D firstPointOnLine = EuclidCoreRandomTools.nextPoint3D(random);
         firstPointOnLine.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D secondPointOnLine = EuclidCoreRandomTools.nextPoint3D(random);
         secondPointOnLine.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(secondPointOnLine, firstPointOnLine);
         // Generate a random vector orthogonal to the line
         Vector3D orthogonalVector = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineDirection, true);
         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         // Generate a random point located at an expected distance from the line
         Point3D query = new Point3D();
         // Randomize on the line
         query.interpolate(firstPointOnLine, secondPointOnLine, EuclidCoreRandomTools.nextDouble(random, 10.0));
         // Move the point away from the line by the expected distance
         query.scaleAdd(expectedDistance, orthogonalVector, query);

         double actualDistance = EuclidGeometryTools.distanceFromPoint3DToLine3D(query, firstPointOnLine, secondPointOnLine);
         assertEquals(expectedDistance, actualDistance, 1.0e-12);

         actualDistance = EuclidGeometryTools.distanceFromPoint3DToLine3D(query, firstPointOnLine, lineDirection);
         assertEquals(expectedDistance, actualDistance, 1.0e-12);

         // Test with a line direction that a magnitude that is too small
         lineDirection.normalize();
         lineDirection.scale(0.999999 * EuclidGeometryTools.ONE_TRILLIONTH);
         expectedDistance = firstPointOnLine.distance(query);
         actualDistance = EuclidGeometryTools.distanceFromPoint3DToLine3D(query, firstPointOnLine, lineDirection);
         assertEquals(expectedDistance, actualDistance, EPSILON);
      }
   }

   @Test
   public void testDistanceFromPoint3DToLineSegment3D() throws Exception
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         Vector3D orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection, true);
         Point3D projection = new Point3D();
         Point3D testPoint = new Point3D();
         double expectedDistance, actualDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentEnd);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testDistanceFromPoint3DToPlane3D() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);
         Point3D secondPointOnPlane = new Point3D();
         secondPointOnPlane.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         Point3D point = new Point3D();
         double scalar = expectedDistance / planeNormal.length();
         if (random.nextBoolean())
            scalar = -scalar;
         point.scaleAdd(scalar, planeNormal, secondPointOnPlane);

         double actualDistance = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnPlane, planeNormal);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         actualDistance = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point.getX(), point.getY(), point.getZ(), pointOnPlane, planeNormal);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testSignedDistanceFromPoint3DToPlane3D() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);
         Point3D secondPointOnPlane = new Point3D();
         secondPointOnPlane.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, -10.0, 10.0);
         Point3D point = new Point3D();
         point.scaleAdd(expectedDistance / planeNormal.length(), planeNormal, secondPointOnPlane);

         double actualDistance = EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(point, pointOnPlane, planeNormal);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testDistanceSquaredFromPoint2DToLineSegment2D() throws Exception
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         EuclidGeometryTools.perpendicularVector2D(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D projection = new Point2D();
         Point2D testPoint = new Point2D();
         double expectedSquaredDistance, actualSquaredDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(testPoint.getX(), testPoint.getY(), lineSegmentStart.getX(),
                                                                                               lineSegmentStart.getY(), lineSegmentEnd.getX(),
                                                                                               lineSegmentEnd.getY());
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(testPoint.getX(), testPoint.getY(), lineSegmentStart,
                                                                                               lineSegmentEnd);
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(testPoint.getX(), testPoint.getY(), lineSegmentStart.getX(),
                                                                                               lineSegmentStart.getY(), lineSegmentEnd.getX(),
                                                                                               lineSegmentEnd.getY());
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(testPoint.getX(), testPoint.getY(), lineSegmentStart,
                                                                                               lineSegmentEnd);
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentEnd);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(testPoint.getX(), testPoint.getY(), lineSegmentStart.getX(),
                                                                                               lineSegmentStart.getY(), lineSegmentEnd.getX(),
                                                                                               lineSegmentEnd.getY());
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(testPoint.getX(), testPoint.getY(), lineSegmentStart,
                                                                                               lineSegmentEnd);
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testDistanceSquaredFromPoint3DToLineSegment3D() throws Exception
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         Vector3D orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection, true);
         Point3D projection = new Point3D();
         Point3D testPoint = new Point3D();
         double expectedSquaredDistance, actualSquaredDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentEnd);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testDoesLineSegment3DIntersectPlane3D() throws Exception
   {
      Random random = new Random(1176L);
      Point3D endPoint0 = new Point3D();
      Point3D endPoint1 = new Point3D();

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);
         Point3D randomLinePlaneIntersection = new Point3D();
         randomLinePlaneIntersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3D lineDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         // Create the two endPoints on each side of the plane:
         endPoint0.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         assertTrue(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertTrue(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Create the two endPoints on one side of the plane:
         endPoint0.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Create the two endPoints on the other side of the plane:
         endPoint0.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Annoying case 1: endPoint0 == endPoint1 => should return false whether the endPoints are on plane or not.
         endPoint0.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.set(endPoint0);
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));
         endPoint0.set(randomLinePlaneIntersection);
         endPoint1.set(endPoint0);
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));
      }

      // Annoying case 2: one of the two endPoints is on the plane, should return false.
      // Tested separately as it is sensitive to numerical errors
      Point3D pointOnPlane = new Point3D();
      Vector3D planeNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D randomLinePlaneIntersection = EuclidCoreRandomTools.nextPoint3D(random);
      randomLinePlaneIntersection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
      randomLinePlaneIntersection.setZ(0.0);

      Vector3D lineDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
      // Ensure that the line direction and the plane normal are somewhat pointing the same direction.
      if (lineDirection.dot(planeNormal) < 0.0)
         lineDirection.negate();

      endPoint0.set(randomLinePlaneIntersection);
      endPoint1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
      assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
      assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));
      endPoint0.set(randomLinePlaneIntersection);
      endPoint1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
      assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
      assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));
   }

   @Test
   public void testDoLine2DAndLineSegment2DIntersect() throws Exception
   {
      Random random = new Random(4353L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing when they're intersecting
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnLine = new Point2D();
         pointOnLine.interpolate(lineSegmentStart, lineSegmentEnd, random.nextDouble());
         Vector2D lineDirection = nextVector2D(random, -10.0, 10.0);
         pointOnLine.scaleAdd(nextDouble(random, 10.0), lineDirection, pointOnLine);

         assertTrue(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart));

         assertTrue(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(),
                                                                          lineSegmentStart, lineSegmentEnd));
         assertTrue(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(),
                                                                          lineSegmentEnd, lineSegmentStart));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing with the line crossing the line supporting the segment but the intersection in outside the line segment.
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnLine = new Point2D();
         pointOnLine.interpolate(lineSegmentStart, lineSegmentEnd, 1.0 + random.nextDouble());
         Vector2D lineDirection = nextVector2D(random, -10.0, 10.0);
         pointOnLine.scaleAdd(nextDouble(random, 10.0), lineDirection, pointOnLine);

         assertFalse(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart));

         assertFalse(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(),
                                                                           lineSegmentStart, lineSegmentEnd));
         assertFalse(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(),
                                                                           lineSegmentEnd, lineSegmentStart));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing with the line and line segment being parallel but not collinear
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         lineDirection.scale(nextDouble(random, 10.0));
         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         Point2D pointOnLine = new Point2D();
         pointOnLine.scaleAdd(nextDouble(random, 0.01, 10.0), orthogonal, lineSegmentStart);
         pointOnLine.scaleAdd(nextDouble(random, 10.0), lineDirection, pointOnLine);

         assertFalse(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart));

         assertFalse(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(),
                                                                           lineSegmentStart, lineSegmentEnd));
         assertFalse(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(),
                                                                           lineSegmentEnd, lineSegmentStart));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Testing with the line and line segment being collinear
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         lineDirection.scale(nextDouble(random, 10.0));
         Point2D pointOnLine = new Point2D(lineSegmentStart);
         pointOnLine.scaleAdd(nextDouble(random, 10.0), lineDirection, pointOnLine);

         assertTrue(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart));

         assertTrue(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(),
                                                                          lineSegmentStart, lineSegmentEnd));
         assertTrue(EuclidGeometryTools.doLine2DAndLineSegment2DIntersect(pointOnLine.getX(), pointOnLine.getY(), lineDirection.getX(), lineDirection.getY(),
                                                                          lineSegmentEnd, lineSegmentStart));
      }
   }

   @Test
   public void testDoLineSegment2DsIntersect() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnLineSegment1 = new Point2D();
         pointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector2D lineDirection2 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection2, pointOnLineSegment1);
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test intersection at one of the end points
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnLineSegment1 = new Point2D(lineSegmentStart1);

         Vector2D lineDirection2 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection2, pointOnLineSegment1);
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if (0.0 < alpha1 && alpha1 < 1.0 || 0.0 < alpha2 && alpha2 < 1.0 || alpha1 * alpha2 < 0.0)
         {
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
         }
         else
         {
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
         }

         // Shift the second line segment such that it becomes only parallel to the first.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test with vertical parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         double x = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D lineSegmentStart1 = new Point2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = new Point2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if (0.0 < alpha1 && alpha1 < 1.0 || 0.0 < alpha2 && alpha2 < 1.0 || alpha1 * alpha2 < 0.0)
         {
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
         }
         else
         {
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
         }

         // Shift the second line segment such that it becomes only parallel to the first.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test with horizontal parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         double y = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D lineSegmentStart1 = new Point2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);
         Point2D lineSegmentEnd1 = new Point2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if (0.0 < alpha1 && alpha1 < 1.0 || 0.0 < alpha2 && alpha2 < 1.0 || alpha1 * alpha2 < 0.0)
         {
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
            assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
         }
         else
         {
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
            assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
         }

         // Shift the second line segment such that it becomes only parallel to the first.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }
   }

   @Test
   public void testDotProduct() throws Exception
   {
      Random random = new Random(32424L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test dotProduct(Point2DReadOnly start1, Point2DReadOnly end1, Point2DReadOnly start2, Point2DReadOnly end2)
         Point2D start1 = EuclidCoreRandomTools.nextPoint2D(random);
         start1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D end1 = EuclidCoreRandomTools.nextPoint2D(random);
         end1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D start2 = EuclidCoreRandomTools.nextPoint2D(random);
         start2.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D end2 = EuclidCoreRandomTools.nextPoint2D(random);
         end2.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D vector1 = new Vector2D();
         vector1.sub(end1, start1);
         Vector2D vector2 = new Vector2D();
         vector2.sub(end2, start2);

         double expectedDotProduct = vector1.dot(vector2);
         double actualDotProduct = EuclidGeometryTools.dotProduct(start1, end1, start2, end2);
         assertEquals(expectedDotProduct, actualDotProduct, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test dotProduct(Point3DReadOnly start1, Point3DReadOnly end1, Point3DReadOnly start2, Point3DReadOnly end2)
         Point3D start1 = EuclidCoreRandomTools.nextPoint3D(random);
         start1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D end1 = EuclidCoreRandomTools.nextPoint3D(random);
         end1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D start2 = EuclidCoreRandomTools.nextPoint3D(random);
         start2.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D end2 = EuclidCoreRandomTools.nextPoint3D(random);
         end2.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector3D vector1 = new Vector3D();
         vector1.sub(end1, start1);
         Vector3D vector2 = new Vector3D();
         vector2.sub(end2, start2);

         double expectedDotProduct = vector1.dot(vector2);
         double actualDotProduct = EuclidGeometryTools.dotProduct(start1, end1, start2, end2);
         assertEquals(expectedDotProduct, actualDotProduct, EPSILON);
      }
   }

   @Test
   public void testIntersectionBetweenLine2DAndBoundingBox2D() throws Exception
   {
      Random random = new Random(4353435L);

      // (No intersections) Test with the line entirely outside 'hovering' over each individual face of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int hoveringAxisIndex = 0; hoveringAxisIndex < 2; hoveringAxisIndex++)
         {
            for (double hoveringDirection = -1.0; hoveringDirection <= 1.0; hoveringDirection += 2.0)
            {
               Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               boundingBoxMax.absolute();
               boundingBoxMax.add(boundingBoxMin);
               Point2D firstPointOnLine = new Point2D();
               Point2D secondPointOnLine = new Point2D();
               Vector2D lineDirection = new Vector2D();

               int otherIndex = (hoveringAxisIndex + 1) % 2;

               Point3D alphaStart = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3D alphaEnd = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

               if (hoveringDirection > 0.0)
               {
                  alphaStart.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line hover outside
                  alphaEnd.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line hover outside
               }
               else
               {
                  alphaStart.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line hover outside
                  alphaEnd.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line hover outside
               }

               if (random.nextBoolean())
               {
                  alphaStart.setElement(otherIndex, 0.0);
                  alphaEnd.setElement(otherIndex, 1.0);
               }
               else
               {
                  alphaStart.setElement(otherIndex, 1.0);
                  alphaEnd.setElement(otherIndex, 0.0);
               }

               firstPointOnLine.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaStart.getX()));
               firstPointOnLine.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaStart.getY()));

               secondPointOnLine.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaEnd.getX()));
               secondPointOnLine.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaEnd.getY()));

               lineDirection.sub(secondPointOnLine, firstPointOnLine);

               Point2D expectedFirstIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D expectedSecondIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D actualFirstIntersection = new Point2D(expectedFirstIntersection);
               Point2D actualSecondIntersection = new Point2D(expectedSecondIntersection);

               int expectedNumberOfIntersections = 0;
               int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                               secondPointOnLine, actualFirstIntersection,
                                                                                                               actualSecondIntersection);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
               EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualFirstIntersection);
               EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualSecondIntersection);
               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                           secondPointOnLine, null, null);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);

               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                           lineDirection, actualFirstIntersection,
                                                                                                           actualSecondIntersection);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
               EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualFirstIntersection);
               EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualSecondIntersection);
               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                           lineDirection, null, null);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
            }
         }
      }

      // (2 intersections) create a point of the line to be inside the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point2D insidePoint = new Point2D();
         insidePoint.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));
         insidePoint.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

         Point2D outsidePoint = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         int index = random.nextInt(2);
         if (random.nextBoolean())
            outsidePoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                       EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)));
         else
            outsidePoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                       EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)));

         Vector2D lineDirection1 = new Vector2D();
         lineDirection1.sub(outsidePoint, insidePoint);
         Vector2D lineDirection2 = new Vector2D();
         lineDirection2.sub(insidePoint, outsidePoint);

         Point2D originalFirstIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D originalSecondIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D firstIntersection = new Point2D(originalFirstIntersection);
         Point2D secondIntersection = new Point2D(originalSecondIntersection);

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, insidePoint, outsidePoint,
                                                                                                   firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         // Test with the points flipped
         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, outsidePoint, insidePoint,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, outsidePoint,
                                                                                                         insidePoint, null, null);
         assertEquals(numberOfIntersections, actualNumberOfIntersections);

         // Test using the line direction
         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, outsidePoint, lineDirection1,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, outsidePoint, lineDirection2,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, insidePoint, lineDirection1,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, insidePoint, lineDirection2,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
      }

      // Assert that is not case where there is only one intersection
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point3D firstPointOnLine = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D secondPointOnLine = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(secondPointOnLine, firstPointOnLine);
         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                   secondPointOnLine, null, null);
         assertNotEquals(1, numberOfIntersections);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, firstPointOnLine, lineDirection,
                                                                                               null, null);
         assertNotEquals(1, numberOfIntersections);
      }
   }

   @Test
   public void testIntersectionBetweenLineSegment2DAndBoundingBox2D() throws Exception
   {
      Random random = new Random(564654L);

      // (No intersections) Test line entirely inside the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alphaStartX = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alphaStartY = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         lineSegmentStart.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaStartX));
         lineSegmentStart.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaStartY));

         double alphaEndX = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alphaEndY = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         lineSegmentEnd.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaEndX));
         lineSegmentEnd.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaEndY));

         Point2D expectedFirstIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D expectedSecondIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D actualFirstIntersection = new Point2D(expectedFirstIntersection);
         Point2D actualSecondIntersection = new Point2D(expectedSecondIntersection);

         int expectedNumberOfIntersections = 0;
         int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax,
                                                                                                                lineSegmentStart, lineSegmentEnd,
                                                                                                                actualFirstIntersection,
                                                                                                                actualSecondIntersection);
         assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualFirstIntersection);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualSecondIntersection);
         actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, lineSegmentStart,
                                                                                                            lineSegmentEnd, null, null);
         assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
      }

      // (No intersections) Test with the line segment entirely outside 'hovering' over each individual face of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            for (double hoveringDirection = -1.0; hoveringDirection <= 1.0; hoveringDirection += 2.0)
            {
               Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               boundingBoxMax.absolute();
               boundingBoxMax.add(boundingBoxMin);
               Point2D lineSegmentStart = new Point2D();
               Point2D lineSegmentEnd = new Point2D();

               Point2D alphaStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               if (hoveringDirection > 0.0)
                  alphaStart.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line segment hover outside
               else
                  alphaStart.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line segment hover outside

               lineSegmentStart.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaStart.getX()));
               lineSegmentStart.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaStart.getY()));

               Point2D alphaEnd = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               if (hoveringDirection > 0.0)
                  alphaEnd.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line segment hover outside
               else
                  alphaEnd.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line segment hover outside
               lineSegmentEnd.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaEnd.getX()));
               lineSegmentEnd.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaEnd.getY()));

               Point2D expectedFirstIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D expectedSecondIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D actualFirstIntersection = new Point2D(expectedFirstIntersection);
               Point2D actualSecondIntersection = new Point2D(expectedSecondIntersection);

               int expectedNumberOfIntersections = 0;
               int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax,
                                                                                                                      lineSegmentStart, lineSegmentEnd,
                                                                                                                      actualFirstIntersection,
                                                                                                                      actualSecondIntersection);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
               EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualFirstIntersection);
               EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualSecondIntersection);
               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax,
                                                                                                                  lineSegmentStart, lineSegmentEnd, null, null);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
            }
         }
      }

      // (Possibly intersecting) Making random bounding box and line segment and asserting the sanity of the results.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         Point2D originalFirstIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D originalSecondIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D firstIntersection = new Point2D(originalFirstIntersection);
         Point2D secondIntersection = new Point2D(originalSecondIntersection);

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, lineSegmentStart,
                                                                                                          lineSegmentEnd, firstIntersection,
                                                                                                          secondIntersection);

         switch (numberOfIntersections)
         {
         case 0:
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(firstIntersection);
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(secondIntersection);
            break;
         case 1:
            assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, lineSegmentStart, lineSegmentEnd, EPSILON);
            assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(secondIntersection);
            break;
         case 2:
            assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, lineSegmentStart, lineSegmentEnd, EPSILON);
            assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            assertPointIsBetweenEndPointsOfLineSegment(secondIntersection, lineSegmentStart, lineSegmentEnd, EPSILON);
            assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            break;
         default:
            fail("Not expecting a number of intersections different than 0, 1, or 2. Got:" + numberOfIntersections);
         }
         int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax,
                                                                                                                lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals(numberOfIntersections, actualNumberOfIntersections);
      }

      // (only one intersection) Making only of the line segment endpoints be inside the bounding box.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point2D insideEndpoint = new Point2D();
         insideEndpoint.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));
         insideEndpoint.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

         Point2D outsideEndpoint = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         int index = random.nextInt(2);
         if (random.nextBoolean())
            outsideEndpoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                          EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)));
         else
            outsideEndpoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                          EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)));

         Point2D originalFirstIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D originalSecondIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D firstIntersection = new Point2D(originalFirstIntersection);
         Point2D secondIntersection = new Point2D(originalSecondIntersection);

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, insideEndpoint,
                                                                                                          outsideEndpoint, firstIntersection,
                                                                                                          secondIntersection);

         assertEquals("Was expecting only one intersection", 1, numberOfIntersections);

         assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, insideEndpoint, outsideEndpoint, EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(secondIntersection);

         // Test with the endpoints flipped
         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, outsideEndpoint,
                                                                                                      insideEndpoint, firstIntersection, secondIntersection);

         assertEquals("Was expecting only one intersection", 1, numberOfIntersections);

         assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, insideEndpoint, outsideEndpoint, EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(secondIntersection);

         int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, outsideEndpoint,
                                                                                                                insideEndpoint, null, null);
         assertEquals(numberOfIntersections, actualNumberOfIntersections);
      }

      // (only one intersection) Making only of the line segment endpoints be on one of the bounding box faces.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point2D onFaceEndpoint = new Point2D();
         onFaceEndpoint.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), nextDouble(random, 0.0, 1.0)));
         onFaceEndpoint.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), nextDouble(random, 0.0, 1.0)));

         Point2D outsideEndpoint = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         int index = random.nextInt(2);
         if (random.nextBoolean())
         {
            onFaceEndpoint.setElement(index, boundingBoxMax.getElement(index));
            outsideEndpoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                          nextDouble(random, 1.0, 10.0)));
         }
         else
         {
            onFaceEndpoint.setElement(index, boundingBoxMin.getElement(index));
            outsideEndpoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                          nextDouble(random, -10.0, 0.0)));
         }

         Point2D originalFirstIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D originalSecondIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D firstIntersection = new Point2D(originalFirstIntersection);
         Point2D secondIntersection = new Point2D(originalSecondIntersection);

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, onFaceEndpoint,
                                                                                                          outsideEndpoint, firstIntersection,
                                                                                                          secondIntersection);

         assertEquals("Was expecting only one intersection", 1, numberOfIntersections);

         assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, onFaceEndpoint, outsideEndpoint, EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(secondIntersection);

         // Test with the endpoints flipped
         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, outsideEndpoint,
                                                                                                      onFaceEndpoint, firstIntersection, secondIntersection);

         assertEquals("Was expecting only one intersection", 1, numberOfIntersections);

         assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, onFaceEndpoint, outsideEndpoint, EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(secondIntersection);
      }

      // Assert exception is thrown when providing a bad bounding box.
      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(new Point2D(1.0, 0.0), new Point2D(0.0, 0.0), new Point2D(), new Point2D(), null,
                                                                              null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(new Point2D(0.0, 1.0), new Point2D(0.0, 0.0), new Point2D(), new Point2D(), null,
                                                                              null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(new Point2D(0.0, 0.0), new Point2D(-1.0, 0.0), new Point2D(), new Point2D(), null,
                                                                              null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(new Point2D(0.0, 0.0), new Point2D(0.0, -1.0), new Point2D(), new Point2D(), null,
                                                                              null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      // Asserts no exception is thrown when the bounding box coordinates are equal
      EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D(new Point2D(0.0, 0.0), new Point2D(0.0, 0.0), new Point2D(), new Point2D(), null,
                                                                           null);
   }

   private void assertPointIsOnBoundingBoxFace(Point2D query, Point2D boundingBoxMin, Point2D boundingBoxMax, double epsilon)
   {
      for (int i = 0; i < 2; i++)
      {
         if (Math.abs(query.getElement(i) - boundingBoxMin.getElement(i)) < epsilon || Math.abs(query.getElement(i) - boundingBoxMax.getElement(i)) < epsilon)
         {
            int nextIndex = (i + 1) % 2;
            assertTrue(boundingBoxMin.getElement(nextIndex) < query.getElement(nextIndex)
                  && query.getElement(nextIndex) < boundingBoxMax.getElement(nextIndex));
            return;
         }
      }
      fail("The query does not belong to any face of the bounding box.");
   }

   private void assertPointIsBetweenEndPointsOfLineSegment(Point2D query, Point2D lineSegmentStart, Point2D lineSegmentEnd, double epsilon)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment2D(query, lineSegmentStart, lineSegmentEnd);
      assertTrue(percentage >= -epsilon && percentage <= 1.0 + epsilon);
      double distance = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(query, lineSegmentStart, lineSegmentEnd);
      assertEquals(0.0, distance, epsilon);
   }

   @Test
   public void testIntersectionBetweenLine2DAndLineSegment2D() throws Exception
   {
      double epsilon = 10.0 * EPSILON;
      Random random = new Random(23423L);
      Point2D actualIntersection = new Point2D();
      boolean success;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Point2D pointOnLine = new Point2D(expectedIntersection);
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         // Expecting intersection
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Make the intersection happen outside the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineLineIntersection = new Point2D();
         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 1.0, 2.0));
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertFalse(success);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualIntersection);
         actualIntersection.setToZero();
         assertNull(EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd));

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertFalse(success);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualIntersection);
         actualIntersection.setToZero();
         assertNull(EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart));

         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, -1.0, 0.0));
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertFalse(success);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualIntersection);
         actualIntersection.setToZero();
         assertNull(EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd));

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertFalse(success);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualIntersection);
         actualIntersection.setToZero();
         assertNull(EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Make the intersection happen on each end point of the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.set(lineSegmentStart);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         expectedIntersection.set(lineSegmentEnd);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Make the line segment and the line parallel not collinear.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         Vector2D orthogonal = new Vector2D(-lineDirection.getY(), lineDirection.getY());

         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), orthogonal, pointOnLine);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertFalse(success);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualIntersection);
         actualIntersection.setToZero();
         assertNull(EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd));
      }

      // Make the line segment and the line collinear.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentStart, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentStart, actualIntersection, epsilon);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentEnd, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentEnd, actualIntersection, epsilon);
      }
   }

   @Test
   public void testIntersectionBetweenLine3DAndBoundingBox3D() throws Exception
   {
      Random random = new Random(4353435L);

      // (No intersections) Test with the line entirely outside 'hovering' over each individual face of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int hoveringAxisIndex = 0; hoveringAxisIndex < 3; hoveringAxisIndex++)
         {
            for (double hoveringDirection = -1.0; hoveringDirection <= 1.0; hoveringDirection += 2.0)
            {
               Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               boundingBoxMax.absolute();
               boundingBoxMax.add(boundingBoxMin);
               Point3D firstPointOnLine = new Point3D();
               Point3D secondPointOnLine = new Point3D();
               Vector3D lineDirection = new Vector3D();

               int otherIndex = (hoveringAxisIndex + random.nextInt(2) + 1) % 3;

               Point3D alphaStart = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3D alphaEnd = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

               if (hoveringDirection > 0.0)
               {
                  alphaStart.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line hover outside
                  alphaEnd.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line hover outside
               }
               else
               {
                  alphaStart.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line hover outside
                  alphaEnd.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line hover outside
               }

               if (random.nextBoolean())
               {
                  alphaStart.setElement(otherIndex, 0.0);
                  alphaEnd.setElement(otherIndex, 1.0);
               }
               else
               {
                  alphaStart.setElement(otherIndex, 1.0);
                  alphaEnd.setElement(otherIndex, 0.0);
               }

               firstPointOnLine.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaStart.getX()));
               firstPointOnLine.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaStart.getY()));
               firstPointOnLine.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), alphaStart.getZ()));

               secondPointOnLine.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaEnd.getX()));
               secondPointOnLine.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaEnd.getY()));
               secondPointOnLine.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), alphaEnd.getZ()));

               lineDirection.sub(secondPointOnLine, firstPointOnLine);

               Point3DBasics expectedFirstIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3DBasics expectedSecondIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3DBasics actualFirstIntersection = new Point3D(expectedFirstIntersection);
               Point3DBasics actualSecondIntersection = new Point3D(expectedSecondIntersection);

               int expectedNumberOfIntersections = 0;
               int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                               secondPointOnLine, actualFirstIntersection,
                                                                                                               actualSecondIntersection);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
               EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
               EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                           secondPointOnLine, null, null);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);

               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                           lineDirection, actualFirstIntersection,
                                                                                                           actualSecondIntersection);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
               EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
               EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                           lineDirection, null, null);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
            }
         }
      }

      // (2 intersections) create a point of the line to be inside the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point3D insidePoint = new Point3D();
         insidePoint.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));
         insidePoint.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));
         insidePoint.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

         Point3D outsidePoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         int index = random.nextInt(3);
         if (random.nextBoolean())
            outsidePoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                       EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)));
         else
            outsidePoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                       EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)));

         Vector3D lineDirection1 = new Vector3D();
         lineDirection1.sub(outsidePoint, insidePoint);
         Vector3D lineDirection2 = new Vector3D();
         lineDirection2.sub(insidePoint, outsidePoint);

         Point3D originalFirstIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D originalSecondIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D firstIntersection = new Point3D(originalFirstIntersection);
         Point3D secondIntersection = new Point3D(originalSecondIntersection);

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, insidePoint, outsidePoint,
                                                                                                   firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         // Test with the points flipped
         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, outsidePoint, insidePoint,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, outsidePoint,
                                                                                                         insidePoint, null, null);
         assertEquals(numberOfIntersections, actualNumberOfIntersections);

         // Test using the line direction
         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, outsidePoint, lineDirection1,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, outsidePoint, lineDirection2,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, insidePoint, lineDirection1,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);

         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, insidePoint, lineDirection2,
                                                                                               firstIntersection, secondIntersection);

         assertEquals("Was expecting 2 intersections", 2, numberOfIntersections);

         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(firstIntersection, insidePoint, outsidePoint), EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(secondIntersection, insidePoint, outsidePoint), EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
      }

      // Assert that is not case where there is only one intersection
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point3D firstPointOnLine = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D secondPointOnLine = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(secondPointOnLine, firstPointOnLine);
         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, firstPointOnLine,
                                                                                                   secondPointOnLine, null, null);
         assertNotEquals(1, numberOfIntersections);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, firstPointOnLine, lineDirection,
                                                                                               null, null);
         assertNotEquals(1, numberOfIntersections);
      }
   }

   @Test
   public void testIntersectionBetweenLine3DAndCylinder3D() throws Exception
   {
      Random random = new Random(65226L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with non intersecting line that goes above the cylinder.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D firstPointOnLine = new Point3D(cylinderRadius, 0.0, cylinderTopZ + EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), firstPointOnLine, firstPointOnLine);

         Point3D secondPointOnLine = new Point3D(cylinderRadius, 0.0, cylinderTopZ + EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), secondPointOnLine, secondPointOnLine);

         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(secondPointOnLine, firstPointOnLine);
         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine);
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, secondPointOnLine);
         lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                                secondPointOnLine, firstIntersection, secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                            secondPointOnLine, null, null);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                            lineDirection, firstIntersection, secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with non intersecting line that goes below the cylinder.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D firstPointOnLine = new Point3D(cylinderRadius, 0.0, cylinderBottomZ - EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), firstPointOnLine, firstPointOnLine);

         Point3D secondPointOnLine = new Point3D(cylinderRadius, 0.0, cylinderBottomZ - EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), secondPointOnLine, secondPointOnLine);

         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(secondPointOnLine, firstPointOnLine);
         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine);
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, secondPointOnLine);
         lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                                secondPointOnLine, firstIntersection, secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                            secondPointOnLine, null, null);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                            lineDirection, firstIntersection, secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with non intersecting line that hovers around cylinder part.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D firstPointOnLine = new Point3D(cylinderRadius + EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0), 0.0,
                                                EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), firstPointOnLine, firstPointOnLine);

         Vector3D fromCylinderToPoint = new Vector3D(firstPointOnLine);
         fromCylinderToPoint.setZ(0.0);
         Vector3D lineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, fromCylinderToPoint, true);

         Point3D secondPointOnLine = new Point3D();
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine);
         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine);
         lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                                secondPointOnLine, firstIntersection, secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                            secondPointOnLine, null, null);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                            lineDirection, firstIntersection, secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with intersecting line going through the top and the bottom faces.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D expectedIntersection1 = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderBottomZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection1, expectedIntersection1);

         Point3D expectedIntersection2 = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderTopZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection2, expectedIntersection2);

         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(expectedIntersection2, expectedIntersection1);
         lineDirection.normalize();

         Point3D firstPointOnLine = new Point3D();
         Point3D secondPointOnLine = new Point3D();

         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, expectedIntersection1);
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, firstPointOnLine);
         lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

         Point3D actualIntersection1 = new Point3D();
         Point3D actualIntersection2 = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                                secondPointOnLine, null, null);
         assertEquals(2, numberOfIntersections);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                            lineDirection, null, null);
         assertEquals(2, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, secondPointOnLine,
                                                                    actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, EPSILON);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, lineDirection,
                                                                    actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, EPSILON);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, secondPointOnLine, firstPointOnLine,
                                                                    actualIntersection2, actualIntersection1);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, EPSILON);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         lineDirection.negate();
         EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, lineDirection,
                                                                    actualIntersection2, actualIntersection1);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting line going through the cylinder part without touching the top and bottom faces.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D expectedIntersection1 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection1, expectedIntersection1);

            Point3D expectedIntersection2 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection2, expectedIntersection2);

            Vector3D lineDirection = new Vector3D();
            lineDirection.sub(expectedIntersection2, expectedIntersection1);
            lineDirection.normalize();

            Point3D firstPointOnLine = new Point3D();
            Point3D secondPointOnLine = new Point3D();

            firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, expectedIntersection1);
            secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, firstPointOnLine);
            lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   firstPointOnLine, secondPointOnLine, null, null);
            assertEquals(2, numberOfIntersections);
            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                               lineDirection, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, secondPointOnLine,
                                                                       actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, LARGE_EPSILON);
            errors.add(expectedIntersection1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, LARGE_EPSILON);
            errors.add(expectedIntersection2.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, lineDirection,
                                                                       actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, LARGE_EPSILON);
            errors.add(expectedIntersection1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, LARGE_EPSILON);
            errors.add(expectedIntersection2.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, secondPointOnLine, firstPointOnLine,
                                                                       actualIntersection2, actualIntersection1);
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, LARGE_EPSILON);
            errors.add(expectedIntersection1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, LARGE_EPSILON);
            errors.add(expectedIntersection2.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            lineDirection.negate();
            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, lineDirection,
                                                                       actualIntersection2, actualIntersection1);
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, LARGE_EPSILON);
            errors.add(expectedIntersection1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, LARGE_EPSILON);
            errors.add(expectedIntersection2.distance(actualIntersection2));
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting line going through the cylinder part once and through the top face.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnTop = new Point3D(EuclidCoreRandomTools.nextDouble(random, 0.0, cylinderRadius), 0.0, cylinderTopZ);
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnTop, pointOnTop);

            Point3D pointOnCylinder = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder, pointOnCylinder);

            Vector3D lineDirection = new Vector3D();
            lineDirection.sub(pointOnCylinder, pointOnTop);
            lineDirection.normalize();

            Point3D firstPointOnLine = new Point3D();
            Point3D secondPointOnLine = new Point3D();

            firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointOnTop);
            secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, firstPointOnLine);
            lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   firstPointOnLine, secondPointOnLine, null, null);
            assertEquals(2, numberOfIntersections);
            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                               lineDirection, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, secondPointOnLine,
                                                                       actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, lineDirection,
                                                                       actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, secondPointOnLine, firstPointOnLine,
                                                                       actualIntersection2, actualIntersection1);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            lineDirection.negate();
            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, lineDirection,
                                                                       actualIntersection2, actualIntersection1);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting line going through the cylinder part once and through the bottom face.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnBottom = new Point3D(EuclidCoreRandomTools.nextDouble(random, 0.0, cylinderRadius), 0.0, cylinderBottomZ);
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnBottom, pointOnBottom);

            Point3D pointOnCylinder = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder, pointOnCylinder);

            Vector3D lineDirection = new Vector3D();
            lineDirection.sub(pointOnCylinder, pointOnBottom);
            lineDirection.normalize();

            Point3D firstPointOnLine = new Point3D();
            Point3D secondPointOnLine = new Point3D();

            firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointOnBottom);
            secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, firstPointOnLine);
            lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   firstPointOnLine, secondPointOnLine, null, null);
            assertEquals(2, numberOfIntersections);
            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine,
                                                                                               lineDirection, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, secondPointOnLine,
                                                                       actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, lineDirection,
                                                                       actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, secondPointOnLine, firstPointOnLine,
                                                                       actualIntersection2, actualIntersection1);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            lineDirection.negate();
            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, firstPointOnLine, lineDirection,
                                                                       actualIntersection2, actualIntersection1);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }
   }

   @Test
   public void testIntersectionBetweenLine3DAndEllipsoid3D() throws Exception
   {
      Random random = new Random(7654L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Non-intersecting line
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);

         Point3D pointOnEllipsoid = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         double sqrtSumOfSquares = Math.sqrt(EuclidCoreTools.normSquared(pointOnEllipsoid.getX() / radiusX, pointOnEllipsoid.getY() / radiusY,
                                                                         pointOnEllipsoid.getZ() / radiusZ));
         pointOnEllipsoid.scale(1.0 / sqrtSumOfSquares);

         Vector3D normalAtPoint = new Vector3D(pointOnEllipsoid);
         normalAtPoint.scale(1.0 / (radiusX * radiusX), 1.0 / (radiusY * radiusY), 1.0 / (radiusZ * radiusZ));
         normalAtPoint.normalize();

         Point3D firstPointOnLine = new Point3D();
         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0001, 10.0), normalAtPoint, pointOnEllipsoid);

         Vector3D lineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normalAtPoint, true);
         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine);
         Point3D secondPointOnLine = new Point3D();
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, firstPointOnLine);

         Point3D intersection1 = new Point3D();
         Point3D intersection2 = new Point3D();
         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX, radiusY, radiusZ, firstPointOnLine, secondPointOnLine,
                                                                                                 intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX, radiusY, radiusZ, firstPointOnLine, lineDirection,
                                                                                             intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX, radiusY, radiusZ, firstPointOnLine, secondPointOnLine,
                                                                                             null, null);
         assertEquals(0, numberOfIntersections);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX, radiusY, radiusZ, firstPointOnLine, lineDirection, null,
                                                                                             null);
         assertEquals(0, numberOfIntersections);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Intersecting line
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);

         Point3D pointOnEllipsoid1 = EuclidCoreRandomTools.nextPoint3D(random, 1.0, 10.0);
         double sqrtSumOfSquares = Math.sqrt(EuclidCoreTools.normSquared(pointOnEllipsoid1.getX() / radiusX, pointOnEllipsoid1.getY() / radiusY,
                                                                         pointOnEllipsoid1.getZ() / radiusZ));
         pointOnEllipsoid1.scale(1.0 / sqrtSumOfSquares);

         Point3D pointOnEllipsoid2 = EuclidCoreRandomTools.nextPoint3D(random, 1.0, 10.0);
         sqrtSumOfSquares = Math.sqrt(EuclidCoreTools.normSquared(pointOnEllipsoid2.getX() / radiusX, pointOnEllipsoid2.getY() / radiusY,
                                                                  pointOnEllipsoid2.getZ() / radiusZ));
         pointOnEllipsoid2.scale(1.0 / sqrtSumOfSquares);

         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(pointOnEllipsoid2, pointOnEllipsoid1);
         lineDirection.normalize();

         Point3D firstPointOnLine = new Point3D();
         Point3D secondPointOnLine = new Point3D();

         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointOnEllipsoid1);
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0), lineDirection, firstPointOnLine);

         Point3D intersection1 = new Point3D();
         Point3D intersection2 = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX, radiusY, radiusZ, firstPointOnLine, secondPointOnLine,
                                                                                                 intersection1, intersection2);
         assertEquals(2, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid1, intersection1, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection2, LARGE_EPSILON);
         intersection1.setToNaN();
         intersection2.setToNaN();

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX, radiusY, radiusZ, secondPointOnLine, firstPointOnLine,
                                                                                             intersection1, intersection2);
         assertEquals(2, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid1, intersection2, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection1, LARGE_EPSILON);
         intersection1.setToNaN();
         intersection2.setToNaN();

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX, radiusY, radiusZ, firstPointOnLine, lineDirection,
                                                                                             intersection1, intersection2);
         assertEquals(2, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid1, intersection1, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection2, LARGE_EPSILON);
         intersection1.setToNaN();
         intersection2.setToNaN();

         lineDirection.negate();
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radiusX, radiusY, radiusZ, secondPointOnLine, lineDirection,
                                                                                             intersection1, intersection2);
         assertEquals(2, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid1, intersection2, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection1, LARGE_EPSILON);
         intersection1.setToNaN();
         intersection2.setToNaN();
      }
   }

   @Test
   public void testIntersectionBetweenLine3DAndPlane3D() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D parallelToPlane = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);

         Point3D expectedIntersection = new Point3D();
         expectedIntersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3D lineDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D pointOnLine = new Point3D();
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, expectedIntersection);

         Point3D actualIntersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection);

         double epsilon = EuclidGeometryTools.ONE_TRILLIONTH;
         if (Math.abs(lineDirection.angle(planeNormal)) > Math.PI / 2.0 - 0.001)
            epsilon = 1.0e-11; // Loss of precision when the line direction and the plane normal are almost orthogonal.

         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Try parallel lines to plane
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector3D lineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, false);
         Point3D pointOnLine = new Point3D();
         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointOnPlane);

         Point3D actualIntersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection);
         assertNull(actualIntersection);

         pointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.0), planeNormal, pointOnLine);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection);
         assertNull(actualIntersection);
      }
   }

   @Test
   public void testIntersectionBetweenLineSegment3DAndBoundingBox3D() throws Exception
   {
      Random random = new Random(564654L);

      // (No intersections) Test line entirely inside the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point3D lineSegmentStart = new Point3D();
         Point3D lineSegmentEnd = new Point3D();

         double alphaStartX = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alphaStartY = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alphaStartZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         lineSegmentStart.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaStartX));
         lineSegmentStart.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaStartY));
         lineSegmentStart.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), alphaStartZ));

         double alphaEndX = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alphaEndY = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         double alphaEndZ = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         lineSegmentEnd.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaEndX));
         lineSegmentEnd.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaEndY));
         lineSegmentEnd.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), alphaEndZ));

         Point3DBasics expectedFirstIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3DBasics expectedSecondIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3DBasics actualFirstIntersection = new Point3D(expectedFirstIntersection);
         Point3DBasics actualSecondIntersection = new Point3D(expectedSecondIntersection);

         int expectedNumberOfIntersections = 0;
         int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax,
                                                                                                                lineSegmentStart, lineSegmentEnd,
                                                                                                                actualFirstIntersection,
                                                                                                                actualSecondIntersection);
         assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
         actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, lineSegmentStart,
                                                                                                            lineSegmentEnd, null, null);
         assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
      }

      // (No intersections) Test with the line segment entirely outside 'hovering' over each individual face of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            for (double hoveringDirection = -1.0; hoveringDirection <= 1.0; hoveringDirection += 2.0)
            {
               Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               boundingBoxMax.absolute();
               boundingBoxMax.add(boundingBoxMin);
               Point3D lineSegmentStart = new Point3D();
               Point3D lineSegmentEnd = new Point3D();

               Point3D alphaStart = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               if (hoveringDirection > 0.0)
                  alphaStart.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line segment hover outside
               else
                  alphaStart.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line segment hover outside

               lineSegmentStart.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaStart.getX()));
               lineSegmentStart.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaStart.getY()));
               lineSegmentStart.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), alphaStart.getZ()));

               Point3D alphaEnd = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               if (hoveringDirection > 0.0)
                  alphaEnd.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line segment hover outside
               else
                  alphaEnd.setElement(axisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line segment hover outside
               lineSegmentEnd.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaEnd.getX()));
               lineSegmentEnd.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaEnd.getY()));
               lineSegmentEnd.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), alphaEnd.getZ()));

               Point3DBasics expectedFirstIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3DBasics expectedSecondIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3DBasics actualFirstIntersection = new Point3D(expectedFirstIntersection);
               Point3DBasics actualSecondIntersection = new Point3D(expectedSecondIntersection);

               int expectedNumberOfIntersections = 0;
               int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax,
                                                                                                                      lineSegmentStart, lineSegmentEnd,
                                                                                                                      actualFirstIntersection,
                                                                                                                      actualSecondIntersection);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
               EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
               EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax,
                                                                                                                  lineSegmentStart, lineSegmentEnd, null, null);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
            }
         }
      }

      // (Possibly intersecting) Making random bounding box and line segment and asserting the sanity of the results.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point3D lineSegmentStart = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D lineSegmentEnd = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

         Point3D originalFirstIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D originalSecondIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D firstIntersection = new Point3D(originalFirstIntersection);
         Point3D secondIntersection = new Point3D(originalSecondIntersection);

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, lineSegmentStart,
                                                                                                          lineSegmentEnd, firstIntersection,
                                                                                                          secondIntersection);

         switch (numberOfIntersections)
         {
         case 0:
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);
            break;
         case 1:
            assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, lineSegmentStart, lineSegmentEnd, EPSILON);
            assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);
            break;
         case 2:
            assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, lineSegmentStart, lineSegmentEnd, EPSILON);
            assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            assertPointIsBetweenEndPointsOfLineSegment(secondIntersection, lineSegmentStart, lineSegmentEnd, EPSILON);
            assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            break;
         default:
            fail("Not expecting a number of intersections different than 0, 1, or 2. Got:" + numberOfIntersections);
         }
         int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax,
                                                                                                                lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals(numberOfIntersections, actualNumberOfIntersections);
      }

      // (only one intersection) Making only of the line segment endpoints be inside the bounding box.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point3D insideEndpoint = new Point3D();
         insideEndpoint.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));
         insideEndpoint.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));
         insideEndpoint.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

         Point3D outsideEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         int index = random.nextInt(3);
         if (random.nextBoolean())
            outsideEndpoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                          EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)));
         else
            outsideEndpoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                          EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)));

         Point3D originalFirstIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D originalSecondIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D firstIntersection = new Point3D(originalFirstIntersection);
         Point3D secondIntersection = new Point3D(originalSecondIntersection);

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, insideEndpoint,
                                                                                                          outsideEndpoint, firstIntersection,
                                                                                                          secondIntersection);

         assertEquals("Was expecting only one intersection", 1, numberOfIntersections);

         assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, insideEndpoint, outsideEndpoint, EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         // Test with the endpoints flipped
         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, outsideEndpoint,
                                                                                                      insideEndpoint, firstIntersection, secondIntersection);

         assertEquals("Was expecting only one intersection", 1, numberOfIntersections);

         assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, insideEndpoint, outsideEndpoint, EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, outsideEndpoint,
                                                                                                                insideEndpoint, null, null);
         assertEquals(numberOfIntersections, actualNumberOfIntersections);
      }

      // (only one intersection) Making only of the line segment endpoints be on one of the bounding box faces.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);
         Point3D onFaceEndpoint = new Point3D();
         onFaceEndpoint.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), nextDouble(random, 0.0, 1.0)));
         onFaceEndpoint.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), nextDouble(random, 0.0, 1.0)));
         onFaceEndpoint.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), nextDouble(random, 0.0, 1.0)));

         Point3D outsideEndpoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         int index = random.nextInt(3);
         if (random.nextBoolean())
         {
            onFaceEndpoint.setElement(index, boundingBoxMax.getElement(index));
            outsideEndpoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                          nextDouble(random, 1.0, 10.0)));
         }
         else
         {
            onFaceEndpoint.setElement(index, boundingBoxMin.getElement(index));
            outsideEndpoint.setElement(index, EuclidCoreTools.interpolate(boundingBoxMin.getElement(index), boundingBoxMax.getElement(index),
                                                                          nextDouble(random, -10.0, 0.0)));
         }

         Point3D originalFirstIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D originalSecondIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D firstIntersection = new Point3D(originalFirstIntersection);
         Point3D secondIntersection = new Point3D(originalSecondIntersection);

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, onFaceEndpoint,
                                                                                                          outsideEndpoint, firstIntersection,
                                                                                                          secondIntersection);

         assertEquals("Was expecting only one intersection", 1, numberOfIntersections);

         assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, onFaceEndpoint, outsideEndpoint, EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         // Test with the endpoints flipped
         firstIntersection.set(originalFirstIntersection);
         secondIntersection.set(originalSecondIntersection);
         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, outsideEndpoint,
                                                                                                      onFaceEndpoint, firstIntersection, secondIntersection);

         assertEquals("Was expecting only one intersection", 1, numberOfIntersections);

         assertPointIsBetweenEndPointsOfLineSegment(firstIntersection, onFaceEndpoint, outsideEndpoint, EPSILON);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);
      }

      // Assert exception is thrown when providing a bad bounding box.
      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(new Point3D(1.0, 0.0, 0.0), new Point3D(0.0, 0.0, 0.0), new Point3D(),
                                                                              new Point3D(), null, null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(new Point3D(0.0, 1.0, 0.0), new Point3D(0.0, 0.0, 0.0), new Point3D(),
                                                                              new Point3D(), null, null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(new Point3D(0.0, 0.0, 1.0), new Point3D(0.0, 0.0, 0.0), new Point3D(),
                                                                              new Point3D(), null, null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(new Point3D(0.0, 0.0, 0.0), new Point3D(-1.0, 0.0, 0.0), new Point3D(),
                                                                              new Point3D(), null, null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, -1.0, 0.0), new Point3D(),
                                                                              new Point3D(), null, null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, -1.0), new Point3D(),
                                                                              new Point3D(), null, null);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      // Asserts the no exception is thrown when the bounding box coordinates are equal
      EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, 0.0), new Point3D(), new Point3D(),
                                                                           null, null);
   }

   private void assertPointIsOnBoundingBoxFace(Point3D query, Point3D boundingBoxMin, Point3D boundingBoxMax, double epsilon)
   {
      for (int i = 0; i < 3; i++)
      {
         if (Math.abs(query.getElement(i) - boundingBoxMin.getElement(i)) < epsilon || Math.abs(query.getElement(i) - boundingBoxMax.getElement(i)) < epsilon)
         {
            int nextIndex = (i + 1) % 3;
            assertTrue(boundingBoxMin.getElement(nextIndex) < query.getElement(nextIndex)
                  && query.getElement(nextIndex) < boundingBoxMax.getElement(nextIndex));
            nextIndex = (nextIndex + 1) % 3;
            assertTrue(boundingBoxMin.getElement(nextIndex) < query.getElement(nextIndex)
                  && query.getElement(nextIndex) < boundingBoxMax.getElement(nextIndex));
            return;
         }
      }
      fail("The query does not belong to any face of the bounding box.");
   }

   private void assertPointIsBetweenEndPointsOfLineSegment(Point3D query, Point3D lineSegmentStart, Point3D lineSegmentEnd, double epsilon)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(query, lineSegmentStart, lineSegmentEnd);
      assertTrue(percentage >= -epsilon && percentage <= 1.0 + epsilon);
      double distance = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(query, lineSegmentStart, lineSegmentEnd);
      assertEquals(0.0, distance, epsilon);
   }

   @Test
   public void testIntersectionBetweenLineSegment3DAndCylinder3D() throws Exception
   {
      Random random = new Random(65226L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with non intersecting line segment that goes above the cylinder.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D lineSegmentStart = new Point3D(cylinderRadius, 0.0, cylinderTopZ + EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), lineSegmentStart, lineSegmentStart);

         Point3D lineSegmentEnd = new Point3D(cylinderRadius, 0.0, cylinderTopZ + EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), lineSegmentEnd, lineSegmentEnd);

         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineSegmentDirection, lineSegmentStart);
         lineSegmentEnd.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineSegmentDirection, lineSegmentEnd);
         lineSegmentDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                       lineSegmentStart, lineSegmentEnd, firstIntersection,
                                                                                                       secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with non intersecting line segment that goes below the cylinder.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D lineSegmentStart = new Point3D(cylinderRadius, 0.0, cylinderBottomZ - EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), lineSegmentStart, lineSegmentStart);

         Point3D lineSegmentEnd = new Point3D(cylinderRadius, 0.0, cylinderBottomZ - EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), lineSegmentEnd, lineSegmentEnd);

         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineSegmentDirection, lineSegmentStart);
         lineSegmentEnd.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineSegmentDirection, lineSegmentEnd);
         lineSegmentDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                       lineSegmentStart, lineSegmentEnd, firstIntersection,
                                                                                                       secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with non intersecting line segment that hovers around cylinder part.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D lineSegmentStart = new Point3D(cylinderRadius + EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0), 0.0,
                                                EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), lineSegmentStart, lineSegmentStart);

         Vector3D fromCylinderToPoint = new Vector3D(lineSegmentStart);
         fromCylinderToPoint.setZ(0.0);
         Vector3D lineSegmentDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, fromCylinderToPoint, true);

         Point3D lineSegmentEnd = new Point3D();
         lineSegmentEnd.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineSegmentDirection, lineSegmentStart);
         lineSegmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineSegmentDirection, lineSegmentStart);
         lineSegmentDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                       lineSegmentStart, lineSegmentEnd, firstIntersection,
                                                                                                       secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with possibly intersecting line segment going through the top and the bottom faces (testing all possibilities).
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D pointOnBottom = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderBottomZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnBottom, pointOnBottom);

         Point3D pointOnTop = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderTopZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnTop, pointOnTop);

         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(pointOnTop, pointOnBottom);
         lineSegmentDirection.normalize();

         Point3D lineSegmentStart = new Point3D();
         Point3D lineSegmentEnd = new Point3D();

         Point3D actualIntersection1 = new Point3D();
         Point3D actualIntersection2 = new Point3D();

         // Line segment entirely going through the cylinder
         lineSegmentStart.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         lineSegmentEnd.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                       lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals(2, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection2, EPSILON);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                           actualIntersection2, actualIntersection1);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection2, EPSILON);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         // Line segment end is inside the cylinder
         lineSegmentStart.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         lineSegmentEnd.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals(1, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         // Line segment start is inside the cylinder
         lineSegmentStart.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         lineSegmentEnd.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals(1, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         // Line segment is before the cylinder
         lineSegmentStart.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         lineSegmentEnd.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals(0, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         // Line segment is after the cylinder
         lineSegmentStart.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         lineSegmentEnd.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals(0, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         // Line segment is inside the cylinder
         lineSegmentStart.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         lineSegmentEnd.interpolate(pointOnBottom, pointOnTop, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                   lineSegmentStart, lineSegmentEnd, null, null);
         assertEquals(0, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();

         EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                           actualIntersection1, actualIntersection2);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with possibly intersecting line segment going through the cylinder part without touching the top and bottom faces (testing all possibilities).
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnCylinder1 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder1, pointOnCylinder1);

            Point3D pointOnCylinder2 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder2, pointOnCylinder2);

            Point3D lineSegmentStart = new Point3D();
            Point3D lineSegmentEnd = new Point3D();

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            // Line segment entirely going through the cylinder
            lineSegmentStart.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            lineSegmentEnd.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                          lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder1, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder2, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder2.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection2, actualIntersection1);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder1, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder2, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder2.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment end is inside the cylinder
            lineSegmentStart.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            lineSegmentEnd.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder1, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder1, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment start is inside the cylinder
            lineSegmentStart.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            lineSegmentEnd.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder2, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder2.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder2, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder2.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment is before cylinder
            lineSegmentStart.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            lineSegmentEnd.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(0, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment is after cylinder
            lineSegmentStart.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
            lineSegmentEnd.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(0, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment is inside cylinder
            lineSegmentStart.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            lineSegmentEnd.interpolate(pointOnCylinder1, pointOnCylinder2, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(0, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting line going through the cylinder part once and through the top face.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnTop = new Point3D(EuclidCoreRandomTools.nextDouble(random, 0.0, cylinderRadius), 0.0, cylinderTopZ);
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnTop, pointOnTop);

            Point3D pointOnCylinder = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder, pointOnCylinder);

            Point3D lineSegmentStart = new Point3D();
            Point3D lineSegmentEnd = new Point3D();

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            // Line segment is entirely going through cylinder
            lineSegmentStart.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            lineSegmentEnd.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                          lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection2, actualIntersection1);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment end is inside cylinder
            lineSegmentStart.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            lineSegmentEnd.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment start is inside cylinder
            lineSegmentStart.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            lineSegmentEnd.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment is before cylinder
            lineSegmentStart.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            lineSegmentEnd.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(0, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment is after cylinder
            lineSegmentStart.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
            lineSegmentEnd.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(0, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment is inside cylinder
            lineSegmentStart.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            lineSegmentEnd.interpolate(pointOnTop, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(0, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with possibly intersecting line going through the cylinder part once and through the bottom face (testing all configurations).
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnBottom = new Point3D(EuclidCoreRandomTools.nextDouble(random, 0.0, cylinderRadius), 0.0, cylinderBottomZ);
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnBottom, pointOnBottom);

            Point3D pointOnCylinder = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder, pointOnCylinder);

            Vector3D lineDirection = new Vector3D();
            lineDirection.sub(pointOnCylinder, pointOnBottom);
            lineDirection.normalize();

            Point3D lineSegmentStart = new Point3D();
            Point3D lineSegmentEnd = new Point3D();

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            // Line segment is entirely going through cylinder
            lineSegmentStart.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            lineSegmentEnd.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                          lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection2, actualIntersection1);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment end is inside cylinder
            lineSegmentStart.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            lineSegmentEnd.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment start is inside cylinder
            lineSegmentStart.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            lineSegmentEnd.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment is before cylinder
            lineSegmentStart.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
            lineSegmentEnd.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(0, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            // Line segment is after cylinder
            lineSegmentStart.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
            lineSegmentEnd.interpolate(pointOnBottom, pointOnCylinder, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

            numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius,
                                                                                                      lineSegmentStart, lineSegmentEnd, null, null);
            assertEquals(0, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentStart, lineSegmentEnd,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();

            EuclidGeometryTools.intersectionBetweenLineSegment3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, lineSegmentEnd, lineSegmentStart,
                                                                              actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }
   }

   @Test
   public void testIntersectionBetweenLineSegment3DAndEllipsoid3D() throws Exception
   {
      Random random = new Random(23454L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Non-intersecting line segment
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);

         Point3D pointOnEllipsoid = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         double sqrtSumOfSquares = Math.sqrt(EuclidCoreTools.normSquared(pointOnEllipsoid.getX() / radiusX, pointOnEllipsoid.getY() / radiusY,
                                                                         pointOnEllipsoid.getZ() / radiusZ));
         pointOnEllipsoid.scale(1.0 / sqrtSumOfSquares);

         Vector3D normalAtPoint = new Vector3D(pointOnEllipsoid);
         normalAtPoint.scale(1.0 / (radiusX * radiusX), 1.0 / (radiusY * radiusY), 1.0 / (radiusZ * radiusZ));
         normalAtPoint.normalize();

         Point3D lineSegmentStart = new Point3D();
         lineSegmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0001, 10.0), normalAtPoint, pointOnEllipsoid);

         Vector3D lineSegmentDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normalAtPoint, true);
         lineSegmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineSegmentDirection, lineSegmentStart);
         Point3D lineSegmentEnd = new Point3D();
         lineSegmentEnd.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineSegmentDirection, lineSegmentStart);

         Point3D intersection1 = new Point3D();
         Point3D intersection2 = new Point3D();
         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentStart,
                                                                                                        lineSegmentEnd, intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentStart, lineSegmentEnd,
                                                                                                    null, null);
         assertEquals(0, numberOfIntersections);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Possibly intersecting line segment (testing all configurations)
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0);

         Point3D pointOnEllipsoid1 = EuclidCoreRandomTools.nextPoint3D(random, 1.0, 10.0);
         double sqrtSumOfSquares = Math.sqrt(EuclidCoreTools.normSquared(pointOnEllipsoid1.getX() / radiusX, pointOnEllipsoid1.getY() / radiusY,
                                                                         pointOnEllipsoid1.getZ() / radiusZ));
         pointOnEllipsoid1.scale(1.0 / sqrtSumOfSquares);

         Point3D pointOnEllipsoid2 = EuclidCoreRandomTools.nextPoint3D(random, 1.0, 10.0);
         sqrtSumOfSquares = Math.sqrt(EuclidCoreTools.normSquared(pointOnEllipsoid2.getX() / radiusX, pointOnEllipsoid2.getY() / radiusY,
                                                                  pointOnEllipsoid2.getZ() / radiusZ));
         pointOnEllipsoid2.scale(1.0 / sqrtSumOfSquares);

         Point3D intersection1 = new Point3D();
         Point3D intersection2 = new Point3D();
         Point3D lineSegmentStart = new Point3D();
         Point3D lineSegmentEnd = new Point3D();

         // Line segment fully going through the ellipsoid
         lineSegmentStart.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         lineSegmentEnd.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentStart,
                                                                                                        lineSegmentEnd, intersection1, intersection2);
         assertEquals(2, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid1, intersection1, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection2, LARGE_EPSILON);
         intersection1.setToNaN();
         intersection2.setToNaN();

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentEnd, lineSegmentStart,
                                                                                                    intersection1, intersection2);
         assertEquals(2, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid1, intersection2, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection1, LARGE_EPSILON);
         intersection1.setToNaN();
         intersection2.setToNaN();

         // Line segment end inside the ellipsoid
         lineSegmentStart.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         lineSegmentEnd.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentStart, lineSegmentEnd,
                                                                                                    intersection1, intersection2);
         assertEquals(1, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid1, intersection1, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentEnd, lineSegmentStart,
                                                                                                    intersection1, intersection2);
         assertEquals(1, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid1, intersection1, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         // Line segment start inside the ellipsoid
         lineSegmentStart.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         lineSegmentEnd.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentStart, lineSegmentEnd,
                                                                                                    intersection1, intersection2);
         assertEquals(1, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection1, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentEnd, lineSegmentStart,
                                                                                                    intersection1, intersection2);
         assertEquals(1, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection1, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         // Line segment before the ellipsoid
         lineSegmentStart.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         lineSegmentEnd.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentStart, lineSegmentEnd,
                                                                                                    intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentEnd, lineSegmentStart,
                                                                                                    intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         // Line segment after the ellipsoid
         lineSegmentStart.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         lineSegmentEnd.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentStart, lineSegmentEnd,
                                                                                                    intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentEnd, lineSegmentStart,
                                                                                                    intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         // Line segment inside the ellipsoid
         lineSegmentStart.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         lineSegmentEnd.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentStart, lineSegmentEnd,
                                                                                                    intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenLineSegment3DAndEllipsoid3D(radiusX, radiusY, radiusZ, lineSegmentEnd, lineSegmentStart,
                                                                                                    intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();
      }
   }

   @Test
   public void testIntersectionBetweenLineSegment3DAndPlane3D() throws Exception
   {
      Point3D endPoint0 = new Point3D();
      Point3D endPoint1 = new Point3D();

      Random random = new Random(1175L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D parallelToPlane = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);

         Point3D expectedIntersection = new Point3D();
         expectedIntersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3D lineDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 10.0));

         // Expecting an actual intersection
         endPoint0.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection, expectedIntersection);
         Point3D actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, 1.0e-11);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, 1.0e-11);

         // Expecting no intersection
         endPoint0.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0);
         assertNull(actualIntersection);
      }

      // Try parallel lines to plane
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector3D lineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, false);
         endPoint0.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointOnPlane);
         endPoint1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointOnPlane);

         Point3D actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);

         double distanceAwayFromPlane = EuclidCoreRandomTools.nextDouble(random, 1.0);
         endPoint0.scaleAdd(distanceAwayFromPlane, planeNormal, endPoint0);
         endPoint1.scaleAdd(distanceAwayFromPlane, planeNormal, endPoint0);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);
      }
   }

   @Test
   public void testIntersectionBetweenRay2DAndBoundingBox2D() throws Exception
   {
      Random random = new Random(4353435L);

      // (No intersections) Test with the ray entirely outside 'hovering' over each individual face of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int hoveringAxisIndex = 0; hoveringAxisIndex < 2; hoveringAxisIndex++)
         {
            for (double hoveringDirection = -1.0; hoveringDirection <= 1.0; hoveringDirection += 2.0)
            {
               Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               boundingBoxMax.absolute();
               boundingBoxMax.add(boundingBoxMin);
               Point2D rayOrigin = new Point2D();
               Point2D pointOnRay = new Point2D();
               Vector2D rayDirection = new Vector2D();

               int otherIndex = (hoveringAxisIndex + 1) % 2;

               Point2D alphaStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D alphaEnd = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

               if (hoveringDirection > 0.0)
               {
                  alphaStart.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line hover outside
                  alphaEnd.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line hover outside
               }
               else
               {
                  alphaStart.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line hover outside
                  alphaEnd.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line hover outside
               }

               if (random.nextBoolean())
               {
                  alphaStart.setElement(otherIndex, 0.0);
                  alphaEnd.setElement(otherIndex, 1.0);
               }
               else
               {
                  alphaStart.setElement(otherIndex, 1.0);
                  alphaEnd.setElement(otherIndex, 0.0);
               }

               rayOrigin.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaStart.getX()));
               rayOrigin.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaStart.getY()));

               pointOnRay.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaEnd.getX()));
               pointOnRay.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaEnd.getY()));

               rayDirection.sub(pointOnRay, rayOrigin);

               Point2D expectedFirstIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D expectedSecondIntersection = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
               Point2D actualFirstIntersection = new Point2D(expectedFirstIntersection);
               Point2D actualSecondIntersection = new Point2D(expectedSecondIntersection);

               int expectedNumberOfIntersections = 0;
               int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, rayOrigin,
                                                                                                              rayDirection, actualFirstIntersection,
                                                                                                              actualSecondIntersection);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
               EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualFirstIntersection);
               EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(actualSecondIntersection);
               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, rayOrigin,
                                                                                                          rayDirection, null, null);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
            }
         }
      }

      // (no intersections) Creates the ray origin outside close to a face of the bounding box and pointing toward the outside of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            boundingBoxMax.absolute();
            boundingBoxMax.add(boundingBoxMin);

            int nextAxis = (axisIndex + 1) % 2;

            Point2D rayOrigin = new Point2D();
            Vector2D rayDirection = new Vector2D();

            Vector2D axisDirection = new Vector2D();
            axisDirection.setElement(axisIndex, 1.0);
            Vector2D orthogonalToAxis = EuclidGeometryTools.perpendicularVector2D(axisDirection);

            if (random.nextBoolean())
            {
               rayOrigin.setElement(axisIndex, boundingBoxMax.getElement(axisIndex) + random.nextDouble());
               rayDirection.interpolate(axisDirection, orthogonalToAxis, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            }
            else
            {
               rayOrigin.setElement(axisIndex, boundingBoxMin.getElement(axisIndex) - random.nextDouble());
               axisDirection.negate();
               rayDirection.interpolate(axisDirection, orthogonalToAxis, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            }

            rayOrigin.setElement(nextAxis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(nextAxis), boundingBoxMax.getElement(nextAxis),
                                                                       EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, rayOrigin, rayDirection,
                                                                                                     null, null);
            assertEquals(0, numberOfIntersections);
         }
      }

      // (2 intersections) Creates the ray origin outside close to a face of the bounding box and pointing toward the inside of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
            boundingBoxMax.absolute();
            boundingBoxMax.add(boundingBoxMin);

            int nextAxis = (axisIndex + 1) % 2;

            Point2D rayOrigin = new Point2D();
            Vector2D rayDirection = new Vector2D();

            Point2D pointOnRay = new Point2D();

            if (random.nextBoolean())
            {
               rayOrigin.setElement(axisIndex, boundingBoxMax.getElement(axisIndex) + EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
               pointOnRay.setElement(axisIndex, boundingBoxMax.getElement(axisIndex));

            }
            else
            {
               rayOrigin.setElement(axisIndex, boundingBoxMin.getElement(axisIndex) - EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
               pointOnRay.setElement(axisIndex, boundingBoxMin.getElement(axisIndex));
            }

            pointOnRay.setElement(nextAxis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(nextAxis), boundingBoxMax.getElement(nextAxis),
                                                                        EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

            rayOrigin.setElement(nextAxis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(nextAxis), boundingBoxMax.getElement(nextAxis),
                                                                       EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

            rayDirection.sub(pointOnRay, rayOrigin);
            rayDirection.normalize();
            rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0));

            Point2D firstIntersection = new Point2D();
            Point2D secondIntersection = new Point2D();
            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, rayOrigin, rayDirection,
                                                                                                     firstIntersection, secondIntersection);
            assertEquals(2, numberOfIntersections);
            assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            assertPointIsOnRay(firstIntersection, rayOrigin, pointOnRay, EPSILON);
            assertPointIsOnRay(secondIntersection, rayOrigin, pointOnRay, EPSILON);
         }
      }

      // (1 intersection) ray originates from inside the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D boundingBoxMin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D boundingBoxMax = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);

         Point2D rayOrigin = new Point2D();
         for (int axis = 0; axis < 2; axis++)
            rayOrigin.setElement(axis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(axis), boundingBoxMax.getElement(axis),
                                                                   EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D firstIntersection = new Point2D();
         Point2D secondIntersection = new Point2D();
         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D(boundingBoxMin, boundingBoxMax, rayOrigin, rayDirection,
                                                                                                  firstIntersection, secondIntersection);
         assertEquals(1, numberOfIntersections);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnRay(firstIntersection, rayOrigin, rayDirection, EPSILON);
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(secondIntersection);
      }
   }

   private void assertPointIsOnRay(Point2D query, Point2D rayOrigin, Vector2D rayDirection, double epsilon)
   {
      Point2D pointOnRay = new Point2D();
      pointOnRay.add(rayOrigin, rayDirection);
      assertPointIsOnRay(query, rayOrigin, pointOnRay, epsilon);
   }

   private void assertPointIsOnRay(Point2D query, Point2D rayOrigin, Point2D pointOnRay, double epsilon)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment2D(query, rayOrigin, pointOnRay);
      assertTrue(percentage >= -epsilon);
      double distance = EuclidGeometryTools.distanceFromPoint2DToLine2D(query, rayOrigin, pointOnRay);
      assertEquals(0.0, distance, epsilon);
   }

   @Test
   public void testDoRay2DAndLineSegment2DIntersect() throws Exception
   {
      Random random = new Random(116L);

      // Trivial test by positioning the intersection on the ray
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D pointOnRay = new Point2D();
         pointOnRay.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, rayOrigin);

         Vector2D lineSegmentDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alphaStart, alphaEnd;

         // Expecting intersection
         alphaStart = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         alphaEnd = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         lineSegmentStart.scaleAdd(alphaStart, lineSegmentDirection, pointOnRay);
         lineSegmentEnd.scaleAdd(alphaEnd, lineSegmentDirection, pointOnRay);
         assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));

         // Not expecting intersection
         alphaStart = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         alphaEnd = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         lineSegmentStart.scaleAdd(alphaStart, lineSegmentDirection, pointOnRay);
         lineSegmentEnd.scaleAdd(alphaEnd, lineSegmentDirection, pointOnRay);
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Trivial test by positioning the intersection on the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);

         Point2D intersection = new Point2D();
         intersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D rayOrigin = new Point2D();

         // Expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, intersection);
         assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, intersection);
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test intersection at the ray origin
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D lineSegmentDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         // Expecting intersection
         lineSegmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection, rayOrigin);
         assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test intersection at an endpoint of the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);

         Point2D intersection = new Point2D(lineSegmentStart);

         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D rayOrigin = new Point2D();

         // Expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, intersection);
         assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0), rayDirection, intersection);
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test with parallel/collinear ray and line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }
         else
         {
            assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test with vertical parallel/collinear ray and line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         double x = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D rayOrigin = new Point2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = new Vector2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }
         else
         {
            assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }

      // Test with horizontal parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         double y = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D rayOrigin = new Point2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);
         Vector2D rayDirection = new Vector2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertTrue("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }
         else
         {
            assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
            assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart));
      }
   }

   @Test
   public void testIntersectionBetweenRay2DAndLineSegment2D() throws Exception
   {
      Random random = new Random(3242L);

      // Trivial test by positioning the intersection on the ray
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, rayOrigin);

         Vector2D lineSegmentDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alphaStart, alphaEnd;

         // Expecting intersection
         alphaStart = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         alphaEnd = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         lineSegmentStart.scaleAdd(alphaStart, lineSegmentDirection, expectedIntersection);
         lineSegmentEnd.scaleAdd(alphaEnd, lineSegmentDirection, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);

         // Not expecting intersection
         alphaStart = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         alphaEnd = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         lineSegmentStart.scaleAdd(alphaStart, lineSegmentDirection, expectedIntersection);
         lineSegmentEnd.scaleAdd(alphaEnd, lineSegmentDirection, expectedIntersection);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Trivial test by positioning the intersection on the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D rayOrigin = new Point2D();

         // Expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, expectedIntersection);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test intersection at the ray origin
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D lineSegmentDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         // Expecting intersection
         lineSegmentStart.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineSegmentDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineSegmentDirection, rayOrigin);
         assertAllCombinationsOfTwoLineSegmentsIntersection(rayOrigin, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test intersection at an endpoint of the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);

         Point2D expectedIntersection = new Point2D(lineSegmentStart);

         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D rayOrigin = new Point2D();

         // Expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);

         // Not expecting intersection
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0), rayDirection, expectedIntersection);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test with parallel/collinear ray and line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random);
         rayOrigin.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2D(random);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(true, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }
         else
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test with vertical parallel/collinear ray and line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         double x = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D rayOrigin = new Point2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D rayDirection = new Vector2D(x, EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(true, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }
         else
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test with horizontal parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         double y = EuclidCoreRandomTools.nextDouble(random, 10.0);
         Point2D rayOrigin = new Point2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);
         Vector2D rayDirection = new Vector2D(EuclidCoreRandomTools.nextDouble(random, 10.0), y);

         Point2D lineSegmentStart = new Point2D();
         Point2D lineSegmentEnd = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the ray
         lineSegmentStart.scaleAdd(alpha1, rayDirection, rayOrigin);
         lineSegmentEnd.scaleAdd(alpha2, rayDirection, rayOrigin);

         if (0.0 < alpha1 || 0.0 < alpha2 || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(true, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }
         else
         {
            assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
         }

         // Shift the second line segment such that it becomes only parallel to the ray.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(rayDirection, rayOrigin);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart.scaleAdd(distance, orthogonal, lineSegmentStart);
         lineSegmentEnd.scaleAdd(distance, orthogonal, lineSegmentEnd);
         assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(false, rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      }

      // Test with various overlapping with collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D rayOrigin = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D rayDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         // Making four points ordered on the line
         Point2D front1 = new Point2D();
         Point2D front2 = new Point2D();
         Point2D back1 = new Point2D();
         Point2D back2 = new Point2D();

         front1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, rayOrigin);
         front2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, rayOrigin);
         back1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, rayOrigin);
         back2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, rayOrigin);

         boolean success;
         Point2D expectedIntersection = new Point2D();
         Point2D actualIntersection = new Point2D();

         // Line segment fully in front of ray
         expectedIntersection.set(front1);
         success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, front1, front2, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(front2);
         success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, front2, front1, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         // Line segment partially in front of ray
         expectedIntersection.set(front1);
         success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, back1, front1, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(front2);
         success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, front2, back1, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);
      }
   }

   private void assertOnlyExistenceOfIntersectionBetweenRay2DAndAllCombinationsOfLineSegment(boolean intersectionExist, Point2D rayOrigin,
                                                                                             Vector2D rayDirection, Point2D lineSegmentStart,
                                                                                             Point2D lineSegmentEnd)
   {
      boolean success;
      Point2D intersectionThatMayContainOnlyNaNs = new Point2D();
      Point2D actualIntersection;

      success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd,
                                                                             intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentStart, lineSegmentEnd);
      assertTrue(actualIntersection != null == intersectionExist);

      success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart,
                                                                             intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lineSegmentEnd, lineSegmentStart);
      assertTrue(actualIntersection != null == intersectionExist);
   }

   private void assertAllCombinationsOfTwoLineSegmentsIntersection(Point2D expectedIntersection, Point2D rayOrigin, Vector2D rayDirection,
                                                                   Point2D lineSegmentStart, Point2D lineSegmentEnd)
   {
      double epsilon = EuclidGeometryTools.ONE_TRILLIONTH;

      Vector2D direction1 = new Vector2D();
      direction1.sub(rayDirection, rayOrigin);
      Vector2D direction2 = new Vector2D();
      Point2D lss2 = lineSegmentStart;
      Point2D lse2 = lineSegmentEnd;
      direction2.sub(lse2, lss2);

      if (Math.abs(rayDirection.dot(direction2)) > 1.0 - 0.0001)
         epsilon = 1.0e-10;

      boolean success;
      Point2D actualIntersection = new Point2D();

      success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lss2, lse2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lse2, lss2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

      actualIntersection = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lss2, lse2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      actualIntersection = EuclidGeometryTools.intersectionBetweenRay2DAndLineSegment2D(rayOrigin, rayDirection, lse2, lss2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
   }

   @Test
   public void testIntersectionBetweenRay3DAndBoundingBox3D() throws Exception
   {
      Random random = new Random(4353435L);

      // (No intersections) Test with the ray entirely outside 'hovering' over each individual face of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int hoveringAxisIndex = 0; hoveringAxisIndex < 3; hoveringAxisIndex++)
         {
            for (double hoveringDirection = -1.0; hoveringDirection <= 1.0; hoveringDirection += 2.0)
            {
               Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               boundingBoxMax.absolute();
               boundingBoxMax.add(boundingBoxMin);
               Point3D rayOrigin = new Point3D();
               Point3D pointOnRay = new Point3D();
               Vector3D rayDirection = new Vector3D();

               int otherIndex = (hoveringAxisIndex + random.nextInt(2) + 1) % 3;

               Point3D alphaStart = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3D alphaEnd = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

               if (hoveringDirection > 0.0)
               {
                  alphaStart.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line hover outside
                  alphaEnd.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0)); // Make the line hover outside
               }
               else
               {
                  alphaStart.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line hover outside
                  alphaEnd.setElement(hoveringAxisIndex, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0)); // Make the line hover outside
               }

               if (random.nextBoolean())
               {
                  alphaStart.setElement(otherIndex, 0.0);
                  alphaEnd.setElement(otherIndex, 1.0);
               }
               else
               {
                  alphaStart.setElement(otherIndex, 1.0);
                  alphaEnd.setElement(otherIndex, 0.0);
               }

               rayOrigin.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaStart.getX()));
               rayOrigin.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaStart.getY()));
               rayOrigin.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), alphaStart.getZ()));

               pointOnRay.setX(EuclidCoreTools.interpolate(boundingBoxMin.getX(), boundingBoxMax.getX(), alphaEnd.getX()));
               pointOnRay.setY(EuclidCoreTools.interpolate(boundingBoxMin.getY(), boundingBoxMax.getY(), alphaEnd.getY()));
               pointOnRay.setZ(EuclidCoreTools.interpolate(boundingBoxMin.getZ(), boundingBoxMax.getZ(), alphaEnd.getZ()));

               rayDirection.sub(pointOnRay, rayOrigin);

               Point3DBasics expectedFirstIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3DBasics expectedSecondIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
               Point3DBasics actualFirstIntersection = new Point3D(expectedFirstIntersection);
               Point3DBasics actualSecondIntersection = new Point3D(expectedSecondIntersection);

               int expectedNumberOfIntersections = 0;
               int actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, rayOrigin,
                                                                                                              rayDirection, actualFirstIntersection,
                                                                                                              actualSecondIntersection);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
               EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
               EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
               actualNumberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, rayOrigin,
                                                                                                          rayDirection, null, null);
               assertEquals(expectedNumberOfIntersections, actualNumberOfIntersections);
            }
         }
      }

      // (no intersections) Creates the ray origin outside close to a face of the bounding box and pointing toward the outside of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
            Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
            boundingBoxMax.absolute();
            boundingBoxMax.add(boundingBoxMin);

            int nextAxis = (axisIndex + 1) % 3;
            int nextNextAxis = (axisIndex + 2) % 3;

            Point3D rayOrigin = new Point3D();
            Vector3D rayDirection = new Vector3D();

            Vector3D axisDirection = new Vector3D();
            axisDirection.setElement(axisIndex, 1.0);
            Vector3D orthogonalToAxis = EuclidCoreRandomTools.nextOrthogonalVector3D(random, axisDirection, true);

            if (random.nextBoolean())
            {
               rayOrigin.setElement(axisIndex, boundingBoxMax.getElement(axisIndex) + random.nextDouble());
               rayDirection.interpolate(axisDirection, orthogonalToAxis, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            }
            else
            {
               rayOrigin.setElement(axisIndex, boundingBoxMin.getElement(axisIndex) - random.nextDouble());
               axisDirection.negate();
               rayDirection.interpolate(axisDirection, orthogonalToAxis, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
            }

            rayOrigin.setElement(nextAxis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(nextAxis), boundingBoxMax.getElement(nextAxis),
                                                                       EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));
            rayOrigin.setElement(nextNextAxis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(nextNextAxis), boundingBoxMax.getElement(nextNextAxis),
                                                                           EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, rayOrigin, rayDirection,
                                                                                                     null, null);
            assertEquals(0, numberOfIntersections);
         }
      }

      // (2 intersections) Creates the ray origin outside close to a face of the bounding box and pointing toward the inside of the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
            Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
            boundingBoxMax.absolute();
            boundingBoxMax.add(boundingBoxMin);

            int nextAxis = (axisIndex + 1) % 3;
            int nextNextAxis = (axisIndex + 2) % 3;

            Point3D rayOrigin = new Point3D();
            Vector3D rayDirection = new Vector3D();

            Point3D pointOnRay = new Point3D();

            if (random.nextBoolean())
            {
               rayOrigin.setElement(axisIndex, boundingBoxMax.getElement(axisIndex) + EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
               pointOnRay.setElement(axisIndex, boundingBoxMax.getElement(axisIndex));

            }
            else
            {
               rayOrigin.setElement(axisIndex, boundingBoxMin.getElement(axisIndex) - EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
               pointOnRay.setElement(axisIndex, boundingBoxMin.getElement(axisIndex));
            }

            pointOnRay.setElement(nextAxis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(nextAxis), boundingBoxMax.getElement(nextAxis),
                                                                        EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));
            pointOnRay.setElement(nextNextAxis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(nextNextAxis), boundingBoxMax.getElement(nextNextAxis),
                                                                            EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

            rayOrigin.setElement(nextAxis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(nextAxis), boundingBoxMax.getElement(nextAxis),
                                                                       EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));
            rayOrigin.setElement(nextNextAxis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(nextNextAxis), boundingBoxMax.getElement(nextNextAxis),
                                                                           EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

            rayDirection.sub(pointOnRay, rayOrigin);
            rayDirection.normalize();
            rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0));

            Point3D firstIntersection = new Point3D();
            Point3D secondIntersection = new Point3D();
            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, rayOrigin, rayDirection,
                                                                                                     firstIntersection, secondIntersection);
            assertEquals(2, numberOfIntersections);
            assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            assertPointIsOnBoundingBoxFace(secondIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
            assertPointIsOnRay(firstIntersection, rayOrigin, pointOnRay, EPSILON);
            assertPointIsOnRay(secondIntersection, rayOrigin, pointOnRay, EPSILON);
         }
      }

      // (1 intersection) ray originates from inside the bounding box
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D boundingBoxMin = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D boundingBoxMax = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         boundingBoxMax.absolute();
         boundingBoxMax.add(boundingBoxMin);

         Point3D rayOrigin = new Point3D();
         for (int axis = 0; axis < 3; axis++)
            rayOrigin.setElement(axis, EuclidCoreTools.interpolate(boundingBoxMin.getElement(axis), boundingBoxMax.getElement(axis),
                                                                   EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)));

         Vector3D rayDirection = EuclidCoreRandomTools.nextVector3D(random, -10.0, 10.0);

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();
         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBoxMin, boundingBoxMax, rayOrigin, rayDirection,
                                                                                                  firstIntersection, secondIntersection);
         assertEquals(1, numberOfIntersections);
         assertPointIsOnBoundingBoxFace(firstIntersection, boundingBoxMin, boundingBoxMax, EPSILON);
         assertPointIsOnRay(firstIntersection, rayOrigin, rayDirection, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);
      }
   }

   private void assertPointIsOnRay(Point3D query, Point3D rayOrigin, Vector3D rayDirection, double epsilon)
   {
      Point3D pointOnRay = new Point3D();
      pointOnRay.add(rayOrigin, rayDirection);
      assertPointIsOnRay(query, rayOrigin, pointOnRay, epsilon);
   }

   private void assertPointIsOnRay(Point3D query, Point3D rayOrigin, Point3D pointOnRay, double epsilon)
   {
      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(query, rayOrigin, pointOnRay);
      assertTrue(percentage >= -epsilon);
      double distance = EuclidGeometryTools.distanceFromPoint3DToLine3D(query, rayOrigin, pointOnRay);
      assertEquals(0.0, distance, epsilon);
   }

   @Test
   public void testIntersectionBetweenRay3DAndCylinder3D() throws Exception
   {
      Random random = new Random(65226L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with non intersecting ray that goes above the cylinder.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D rayOrigin = new Point3D(cylinderRadius, 0.0, cylinderTopZ + EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), rayOrigin, rayOrigin);

         Point3D pointOnRay = new Point3D(cylinderRadius, 0.0, cylinderTopZ + EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnRay, pointOnRay);

         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnRay, rayOrigin);
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), rayDirection, rayOrigin);
         pointOnRay.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), rayDirection, pointOnRay);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                               rayDirection, firstIntersection, secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                           rayDirection, null, null);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with non intersecting ray that goes below the cylinder.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D rayOrigin = new Point3D(cylinderRadius, 0.0, cylinderBottomZ - EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), rayOrigin, rayOrigin);

         Point3D pointOnRay = new Point3D(cylinderRadius, 0.0, cylinderBottomZ - EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnRay, pointOnRay);

         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnRay, rayOrigin);
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), rayDirection, rayOrigin);
         pointOnRay.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), rayDirection, pointOnRay);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                               rayDirection, firstIntersection, secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                           rayDirection, null, null);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with non intersecting ray that hovers around cylinder part.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D rayOrigin = new Point3D(cylinderRadius + EuclidCoreRandomTools.nextDouble(random, 0.0001, 1.0), 0.0,
                                         EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), rayOrigin, rayOrigin);

         Vector3D fromCylinderToPoint = new Vector3D(rayOrigin);
         fromCylinderToPoint.setZ(0.0);
         Vector3D rayDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, fromCylinderToPoint, true);

         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), rayDirection, rayOrigin);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point3D firstIntersection = new Point3D();
         Point3D secondIntersection = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                               rayDirection, firstIntersection, secondIntersection);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(firstIntersection);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(secondIntersection);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                           rayDirection, null, null);
         assertEquals("Iteration: " + i, 0, numberOfIntersections);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with intersecting ray going entirely through the top and the bottom faces.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D expectedIntersection1 = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderBottomZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection1, expectedIntersection1);

         Point3D expectedIntersection2 = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderTopZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection2, expectedIntersection2);

         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(expectedIntersection2, expectedIntersection1);
         rayDirection.normalize();

         Point3D rayOrigin = new Point3D();

         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, expectedIntersection1);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

         Point3D actualIntersection1 = new Point3D();
         Point3D actualIntersection2 = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                               rayDirection, null, null);
         assertEquals(2, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection, actualIntersection1,
                                                                   actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, EPSILON);
         actualIntersection1.setToNaN();
         actualIntersection2.setToNaN();
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with intersecting ray starting from inside the cylinder and going through the bottom face.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D expectedIntersection = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderBottomZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection, expectedIntersection);

         Point3D pointOnTop = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderTopZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnTop, pointOnTop);

         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(expectedIntersection, pointOnTop);
         rayDirection.normalize();

         Point3D rayOrigin = new Point3D();

         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, expectedIntersection.distance(pointOnTop)), rayDirection, pointOnTop);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

         Point3D actualIntersection1 = new Point3D();
         Point3D actualIntersection2 = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                               rayDirection, null, null);
         assertEquals(1, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection, actualIntersection1,
                                                                   actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with intersecting ray starting from inside the cylinder and going through the top face.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D pointOnBottom = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderBottomZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnBottom, pointOnBottom);

         Point3D expectedIntersection = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderTopZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection, expectedIntersection);

         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(expectedIntersection, pointOnBottom);
         rayDirection.normalize();

         Point3D rayOrigin = new Point3D();

         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, pointOnBottom.distance(expectedIntersection)), rayDirection, pointOnBottom);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

         Point3D actualIntersection1 = new Point3D();
         Point3D actualIntersection2 = new Point3D();

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                               rayDirection, null, null);
         assertEquals(1, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection, actualIntersection1,
                                                                   actualIntersection2);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection1, EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         actualIntersection1.setToNaN();
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Try with intersecting line going entirely through the top and the bottom faces, but with ray pointing away from the cylinder.
         double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
         double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

         Point3D pointOnBottom = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderBottomZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnBottom, pointOnBottom);

         Point3D pointOnTop = new Point3D(EuclidCoreRandomTools.nextDouble(random, cylinderRadius), 0.0, cylinderTopZ);
         RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnTop, pointOnTop);

         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnTop, pointOnBottom);
         rayDirection.normalize();

         Point3D rayOrigin = new Point3D();

         Point3D actualIntersection1 = new Point3D();
         Point3D actualIntersection2 = new Point3D();

         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, pointOnTop);
         rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                               rayDirection, null, null);
         assertEquals(0, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection, actualIntersection1,
                                                                   actualIntersection2);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);

         rayDirection.negate();
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), rayDirection, pointOnBottom);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                           rayDirection, null, null);
         assertEquals(0, numberOfIntersections);

         EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection, actualIntersection1,
                                                                   actualIntersection2);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting ray going entirely through the cylinder part without touching the top and bottom faces.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D expectedIntersection1 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection1, expectedIntersection1);

            Point3D expectedIntersection2 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection2, expectedIntersection2);

            Vector3D rayDirection = new Vector3D();
            rayDirection.sub(expectedIntersection2, expectedIntersection1);
            rayDirection.normalize();

            Point3D rayOrigin = new Point3D();

            rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, expectedIntersection1);
            rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                                  rayDirection, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection,
                                                                      actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, LARGE_EPSILON);
            errors.add(expectedIntersection1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, LARGE_EPSILON);
            errors.add(expectedIntersection2.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting ray starting inside the cylinder and going through the cylinder part without touching the top and bottom faces.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnCylinder1 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder1, pointOnCylinder1);

            Point3D pointOnCylinder2 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder2, pointOnCylinder2);

            Vector3D rayDirection = new Vector3D();
            rayDirection.sub(pointOnCylinder2, pointOnCylinder1);
            rayDirection.normalize();

            Point3D rayOrigin = new Point3D();

            rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, pointOnCylinder1.distance(pointOnCylinder2)), rayDirection, pointOnCylinder1);
            rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                                  rayDirection, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection,
                                                                      actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder2, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder2.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), 1.5 * EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting line going through the cylinder part without touching the top and bottom faces, but with the ray pointing away from the cylinder.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D expectedIntersection1 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection1, expectedIntersection1);

            Point3D expectedIntersection2 = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), expectedIntersection2, expectedIntersection2);

            Vector3D rayDirection = new Vector3D();
            rayDirection.sub(expectedIntersection2, expectedIntersection1);
            rayDirection.normalize();

            Point3D rayOrigin = new Point3D();

            rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, expectedIntersection1);
            rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                                  rayDirection, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection,
                                                                      actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection1, actualIntersection1, LARGE_EPSILON);
            errors.add(expectedIntersection1.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection2, actualIntersection2, LARGE_EPSILON);
            errors.add(expectedIntersection2.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting ray going through the cylinder part once and through the top face.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnTop = new Point3D(EuclidCoreRandomTools.nextDouble(random, 0.0, cylinderRadius), 0.0, cylinderTopZ);
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnTop, pointOnTop);

            Point3D pointOnCylinder = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder, pointOnCylinder);

            Vector3D rayDirection = new Vector3D();
            rayDirection.sub(pointOnCylinder, pointOnTop);
            rayDirection.normalize();

            Point3D rayOrigin = new Point3D();

            rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, pointOnTop);
            rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                                  rayDirection, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection,
                                                                      actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting line going through the cylinder part once and through the top face, but the ray only intersecting one of them.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnTop = new Point3D(EuclidCoreRandomTools.nextDouble(random, 0.0, cylinderRadius), 0.0, cylinderTopZ);
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnTop, pointOnTop);

            Point3D pointOnCylinder = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder, pointOnCylinder);

            Vector3D rayDirection = new Vector3D();
            rayDirection.sub(pointOnCylinder, pointOnTop);
            rayDirection.normalize();

            Point3D rayOrigin = new Point3D();

            rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, pointOnCylinder.distance(pointOnTop)), rayDirection, pointOnTop);
            rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            // Intersect cylinder part only
            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                                  rayDirection, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection,
                                                                      actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();

            // Intersect top face only
            rayDirection.negate();
            numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                              rayDirection, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection,
                                                                      actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnTop, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnTop.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting ray going through the cylinder part once and through the bottom face.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnBottom = new Point3D(EuclidCoreRandomTools.nextDouble(random, 0.0, cylinderRadius), 0.0, cylinderBottomZ);
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnBottom, pointOnBottom);

            Point3D pointOnCylinder = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder, pointOnCylinder);

            Vector3D rayDirection = new Vector3D();
            rayDirection.sub(pointOnCylinder, pointOnBottom);
            rayDirection.normalize();

            Point3D rayOrigin = new Point3D();

            rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), rayDirection, pointOnBottom);
            rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                                   rayDirection, null, null);
            assertEquals(2, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection,
                                                                       actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection2, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection2));
            actualIntersection1.setToNaN();
            actualIntersection2.setToNaN();
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }

      {
         List<Double> errors = new ArrayList<>();

         for (int i = 0; i < ITERATIONS; i++)
         { // Try with intersecting line going through the cylinder part once and through the bottom face, but the ray only intersecting one of them.
            double cylinderRadius = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderBottomZ = EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);
            double cylinderTopZ = cylinderBottomZ + EuclidCoreRandomTools.nextDouble(random, 0.01, 1.0);

            Point3D pointOnBottom = new Point3D(EuclidCoreRandomTools.nextDouble(random, 0.0, cylinderRadius), 0.0, cylinderBottomZ);
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnBottom, pointOnBottom);

            Point3D pointOnCylinder = new Point3D(cylinderRadius, 0.0, EuclidCoreRandomTools.nextDouble(random, cylinderBottomZ, cylinderTopZ));
            RotationMatrixTools.applyYawRotation(EuclidCoreRandomTools.nextDouble(random, Math.PI), pointOnCylinder, pointOnCylinder);

            Vector3D rayDirection = new Vector3D();
            rayDirection.sub(pointOnCylinder, pointOnBottom);
            rayDirection.normalize();

            Point3D rayOrigin = new Point3D();

            rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, pointOnCylinder.distance(pointOnBottom)), rayDirection, pointOnBottom);
            rayDirection.scale(EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0));

            Point3D actualIntersection1 = new Point3D();
            Point3D actualIntersection2 = new Point3D();

            // Intersect cylinder part only
            int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                                  rayDirection, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection,
                                                                      actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnCylinder, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnCylinder.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();

            // Intersect bottom face only
            rayDirection.negate();
            numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin,
                                                                                              rayDirection, null, null);
            assertEquals(1, numberOfIntersections);

            EuclidGeometryTools.intersectionBetweenRay3DAndCylinder3D(cylinderBottomZ, cylinderTopZ, cylinderRadius, rayOrigin, rayDirection,
                                                                      actualIntersection1, actualIntersection2);
            EuclidCoreTestTools.assertTuple3DEquals(pointOnBottom, actualIntersection1, LARGE_EPSILON);
            errors.add(pointOnBottom.distance(actualIntersection1));
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualIntersection2);
            actualIntersection1.setToNaN();
         }

         assertEquals(0.0, errors.stream().collect(Collectors.averagingDouble(Double::doubleValue)), EPSILON);
      }
   }

   @Test
   public void testIntersectionBetweenRay3DAndEllipsoid3D() throws Exception
   {
      Random random = new Random(7654L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Non-intersecting line
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);

         Point3D pointOnEllipsoid = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         double sqrtSumOfSquares = Math.sqrt(EuclidCoreTools.normSquared(pointOnEllipsoid.getX() / radiusX, pointOnEllipsoid.getY() / radiusY,
                                                                         pointOnEllipsoid.getZ() / radiusZ));
         pointOnEllipsoid.scale(1.0 / sqrtSumOfSquares);

         Vector3D normalAtPoint = new Vector3D(pointOnEllipsoid);
         normalAtPoint.scale(1.0 / (radiusX * radiusX), 1.0 / (radiusY * radiusY), 1.0 / (radiusZ * radiusZ));
         normalAtPoint.normalize();

         Point3D rayOrigin = new Point3D();
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0001, 10.0), normalAtPoint, pointOnEllipsoid);

         Vector3D rayDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, normalAtPoint, true);
         rayOrigin.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), rayDirection, rayOrigin);

         Point3D intersection1 = new Point3D();
         Point3D intersection2 = new Point3D();
         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(radiusX, radiusY, radiusZ, rayOrigin, rayDirection,
                                                                                                intersection1, intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(radiusX, radiusY, radiusZ, rayOrigin, rayDirection, null, null);
         assertEquals(0, numberOfIntersections);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Possibly intersecting ray (testing all configurations)
         double radiusX = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusY = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);
         double radiusZ = EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0);

         Point3D pointOnEllipsoid1 = EuclidCoreRandomTools.nextPoint3D(random, 1.0, 10.0);
         double sqrtSumOfSquares = Math.sqrt(EuclidCoreTools.normSquared(pointOnEllipsoid1.getX() / radiusX, pointOnEllipsoid1.getY() / radiusY,
                                                                         pointOnEllipsoid1.getZ() / radiusZ));
         pointOnEllipsoid1.scale(1.0 / sqrtSumOfSquares);

         Point3D pointOnEllipsoid2 = EuclidCoreRandomTools.nextPoint3D(random, 1.0, 10.0);
         sqrtSumOfSquares = Math.sqrt(EuclidCoreTools.normSquared(pointOnEllipsoid2.getX() / radiusX, pointOnEllipsoid2.getY() / radiusY,
                                                                  pointOnEllipsoid2.getZ() / radiusZ));
         pointOnEllipsoid2.scale(1.0 / sqrtSumOfSquares);

         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnEllipsoid2, pointOnEllipsoid1);
         rayDirection.normalize();

         Point3D rayOrigin = new Point3D();
         Point3D intersection1 = new Point3D();
         Point3D intersection2 = new Point3D();

         // Ray entirely goes through the ellipsoid
         rayOrigin.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));

         int numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(radiusX, radiusY, radiusZ, rayOrigin, rayDirection,
                                                                                                intersection1, intersection2);
         assertEquals(2, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid1, intersection1, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection2, LARGE_EPSILON);
         intersection1.setToNaN();
         intersection2.setToNaN();

         // Ray starts from inside the ellipsoid
         rayOrigin.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(radiusX, radiusY, radiusZ, rayOrigin, rayDirection, intersection1,
                                                                                            intersection2);
         assertEquals(1, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DEquals(pointOnEllipsoid2, intersection1, LARGE_EPSILON);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();

         // Ray goes away from the ellipsoid, no intersection
         rayOrigin.interpolate(pointOnEllipsoid1, pointOnEllipsoid2, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));

         numberOfIntersections = EuclidGeometryTools.intersectionBetweenRay3DAndEllipsoid3D(radiusX, radiusY, radiusZ, rayOrigin, rayDirection, intersection1,
                                                                                            intersection2);
         assertEquals(0, numberOfIntersections);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection1);
         EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(intersection2);
         intersection1.setToNaN();
         intersection2.setToNaN();
      }
   }

   @Test
   public void testIntersectionBetweenTwoLine2Ds() throws Exception
   {
      Random random = new Random(1176L);
      double epsilon = EPSILON;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine1 = EuclidCoreRandomTools.nextPoint2D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D lineDirection1 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0));
         Point2D secondPointOnLine1 = new Point2D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, firstPointOnLine1);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, firstPointOnLine1);

         Vector2D lineDirection2 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0));
         Point2D firstPointOnLine2 = new Point2D(expectedIntersection);
         Point2D secondPointOnLine2 = new Point2D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);

         if (Math.abs(lineDirection1.dot(lineDirection2) / lineDirection1.length() / lineDirection2.length()) > 1.0 - 0.0005)
            epsilon = 1.0e-11; // Loss of precision for small angles between the two lines.
         else
            epsilon = 1.0e-12;
         Point2D actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, lineDirection1, firstPointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         firstPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);
         actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, lineDirection1, firstPointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Test when parallel but not collinear
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine1 = EuclidCoreRandomTools.nextPoint2D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D lineDirection1 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0));
         Point2D secondPointOnLine1 = new Point2D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, firstPointOnLine1);

         Vector2D lineDirection2 = new Vector2D(lineDirection1);
         if (random.nextBoolean())
            lineDirection2.negate();
         Point2D firstPointOnLine2 = new Point2D(firstPointOnLine1);
         Point2D secondPointOnLine2 = new Point2D();

         Vector2D orthogonal = new Vector2D(-lineDirection1.getY(), lineDirection1.getX());

         firstPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, firstPointOnLine2);
         firstPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);
         Point2D actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, lineDirection1, firstPointOnLine2, lineDirection2);
         assertNull(actualIntersection);
         actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
         assertNull(actualIntersection);
      }

      // Test when collinear
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine1 = EuclidCoreRandomTools.nextPoint2D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D lineDirection1 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.1, 10.0));
         Point2D secondPointOnLine1 = new Point2D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection1, firstPointOnLine1);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.set(firstPointOnLine1);

         Vector2D lineDirection2 = new Vector2D(lineDirection1);
         Point2D firstPointOnLine2 = new Point2D(expectedIntersection);
         Point2D secondPointOnLine2 = new Point2D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);

         Point2D actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, lineDirection1, firstPointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         firstPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection2, firstPointOnLine2);
         actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, lineDirection1, firstPointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
         actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }
   }

   @Test
   public void testIntersectionBetweenTwoLineSegment2Ds() throws Exception
   {
      Random random = new Random(3242L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart1, lineSegmentEnd1, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));

         Vector2D lineDirection2 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test intersection at one of the end points
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D expectedIntersection = new Point2D(lineSegmentStart1);

         Vector2D lineDirection2 = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = EuclidCoreRandomTools.nextDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.nextDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if (0.0 < alpha1 && alpha1 < 1.0 || 0.0 < alpha2 && alpha2 < 1.0 || alpha1 * alpha2 < 0.0)
         {
            assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(true, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         }
         else
         {
            assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
         }

         // Shift the second line segment such that it becomes only parallel to the first.
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd1, lineSegmentStart1);
         orthogonal.set(-orthogonal.getY(), orthogonal.getX());
         orthogonal.normalize();

         double distance = EuclidCoreRandomTools.nextDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test with various overlapping with collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 1.0);

         // Making four points ordered on the line
         Point2D a = new Point2D();
         Point2D b = new Point2D();
         Point2D c = new Point2D();
         Point2D d = new Point2D();

         double alphaA = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double alphaB = alphaA + EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double alphaC = alphaB + EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         double alphaD = alphaC + EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         a.scaleAdd(alphaA, lineDirection, pointOnLine);
         b.scaleAdd(alphaB, lineDirection, pointOnLine);
         c.scaleAdd(alphaC, lineDirection, pointOnLine);
         d.scaleAdd(alphaD, lineDirection, pointOnLine);

         boolean success;
         Point2D expectedIntersection = new Point2D();
         Point2D actualIntersection = new Point2D();

         // Line segment 1 contains line segment 2
         expectedIntersection.set(b);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(a, d, b, c, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(c);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(a, d, c, b, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(b);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(d, a, b, c, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(c);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(d, a, c, b, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         // Line segment 2 contains line segment 1
         expectedIntersection.set(b);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(b, c, a, d, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(c);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(c, b, a, d, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(b);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(b, c, d, a, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(c);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(c, b, d, a, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         // The line segments partially overlap
         expectedIntersection.set(b);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(a, c, b, d, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(b);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(a, c, d, b, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(b);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(c, a, b, d, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);

         expectedIntersection.set(b);
         success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(c, a, d, b, actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, EPSILON);
      }
   }

   private void assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(boolean intersectionExist, Point2D lineSegmentStart1, Point2D lineSegmentEnd1,
                                                                                Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {
      boolean success;
      Point2D intersectionThatMayContainOnlyNaNs = new Point2D();
      Point2D actualIntersection;

      Point2D lss1 = lineSegmentStart1;
      Point2D lse1 = lineSegmentEnd1;
      Point2D lss2 = lineSegmentStart2;
      Point2D lse = lineSegmentEnd2;
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss1, lse1, lss2, lse, intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss1, lse1, lss2, lse);
      assertTrue(actualIntersection != null == intersectionExist);

      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss1, lse1, lse, lss2, intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss1, lse1, lse, lss2);
      assertTrue(actualIntersection != null == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse1, lss1, lss2, lse, intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse1, lss1, lss2, lse);
      assertTrue(actualIntersection != null == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse1, lss1, lse, lss2, intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse1, lss1, lse, lss2);
      assertTrue(actualIntersection != null == intersectionExist);

      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss2, lse, lss1, lse1, intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss2, lse, lss1, lse1);
      assertTrue(actualIntersection != null == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss2, lse, lse1, lss1, intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss2, lse, lse1, lss1);
      assertTrue(actualIntersection != null == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse, lss2, lss1, lse1, intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse, lss2, lss1, lse1);
      assertTrue(actualIntersection != null == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse, lss2, lse1, lss1, intersectionThatMayContainOnlyNaNs);
      assertTrue(success == intersectionExist);
      if (!intersectionExist)
      {
         EuclidCoreTestTools.assertTuple2DContainsOnlyNaN(intersectionThatMayContainOnlyNaNs);
         intersectionThatMayContainOnlyNaNs.setToZero();
      }
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse, lss2, lse1, lss1);
      assertTrue(actualIntersection != null == intersectionExist);
   }

   private void assertAllCombinationsOfTwoLineSegmentsIntersection(Point2D expectedIntersection, Point2D lineSegmentStart1, Point2D lineSegmentEnd1,
                                                                   Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {
      double epsilon = EuclidGeometryTools.ONE_TRILLIONTH;

      Vector2D direction1 = new Vector2D();
      Point2D lss1 = lineSegmentStart1;
      Point2D lse1 = lineSegmentEnd1;
      direction1.sub(lse1, lss1);
      Vector2D direction2 = new Vector2D();
      Point2D lss2 = lineSegmentStart2;
      Point2D lse2 = lineSegmentEnd2;
      direction2.sub(lse2, lss2);

      if (Math.abs(direction1.dot(direction2)) > 1.0 - 0.0001)
         epsilon = 1.0e-10;

      boolean success;
      Point2D actualIntersection = new Point2D();

      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss1, lse1, lss2, lse2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss1, lse1, lse2, lss2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse1, lss1, lss2, lse2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse1, lss1, lse2, lss2, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss2, lse2, lss1, lse1, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss2, lse2, lse1, lss1, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse2, lss2, lss1, lse1, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse2, lss2, lse1, lss1, actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss1, lse1, lss2, lse2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss1, lse1, lse2, lss2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse1, lss1, lss2, lse2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse1, lss1, lse2, lss2);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss2, lse2, lss1, lse1);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lss2, lse2, lse1, lss1);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse2, lss2, lss1, lse1);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lse2, lss2, lse1, lss1);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
   }

   @Test
   public void testIntersectionBetweenTwoPlane3Ds() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane1 = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal1 = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         Vector3D firstParallelToPlane1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal1, true);
         Vector3D secondParallelToPlane1 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal1, true);

         Point3D firstPointOnIntersection = new Point3D();
         Point3D secondPointOnIntersection = new Point3D();
         firstPointOnIntersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), firstParallelToPlane1, pointOnPlane1);
         secondPointOnIntersection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), secondParallelToPlane1, firstPointOnIntersection);

         Vector3D expectedIntersectionDirection = new Vector3D();
         expectedIntersectionDirection.sub(secondPointOnIntersection, firstPointOnIntersection);
         expectedIntersectionDirection.normalize();

         double rotationAngle = EuclidCoreRandomTools.nextDouble(random, Math.PI);
         AxisAngle rotationAxisAngle = new AxisAngle(expectedIntersectionDirection, rotationAngle);

         Vector3D planeNormal2 = new Vector3D();
         rotationAxisAngle.transform(planeNormal1, planeNormal2);
         planeNormal2.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         Point3D pointOnPlane2 = new Point3D();

         Vector3D parallelToPlane2 = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal2, true);
         pointOnPlane2.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), parallelToPlane2, firstPointOnIntersection);

         Point3D actualPointOnIntersection = new Point3D();
         Vector3D actualIntersectionDirection = new Vector3D();

         boolean success = EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1, planeNormal1, pointOnPlane2, planeNormal2,
                                                                              actualPointOnIntersection, actualIntersectionDirection);
         boolean areParallel = EuclidGeometryTools.areVector3DsParallel(planeNormal1, planeNormal2, 1.0e-6);
         assertNotEquals(areParallel, success);
         if (areParallel)
            continue;

         if (expectedIntersectionDirection.dot(actualIntersectionDirection) < 0.0)
            expectedIntersectionDirection.negate();

         String message = "Angle between vectors " + expectedIntersectionDirection.angle(actualIntersectionDirection);
         assertTrue(message, EuclidGeometryTools.areVector3DsParallel(expectedIntersectionDirection, actualIntersectionDirection, 1.0e-7));
         assertEquals(1.0, actualIntersectionDirection.length(), EPSILON);

         if (planeNormal1.dot(planeNormal2) < 0.0)
            planeNormal1.negate();

         double epsilon = 1.0e-9;

         if (planeNormal1.angle(planeNormal2) < 0.15)
            epsilon = 1.0e-8;
         if (planeNormal1.angle(planeNormal2) < 0.05)
            epsilon = 1.0e-7;
         if (planeNormal1.angle(planeNormal2) < 0.03)
            epsilon = 1.0e-6;
         if (planeNormal1.angle(planeNormal2) < 0.001)
            epsilon = 1.0e-5;
         if (planeNormal1.angle(planeNormal2) < 0.0001)
            epsilon = 1.0e-4;
         if (planeNormal1.angle(planeNormal2) < 0.00005)
            epsilon = 1.0e-3;

         //         System.out.println("angle: " + planeNormal1.angle(planeNormal2) + ", distance: " + pointOnPlane1.distance(pointOnPlane2));
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint3DToLine3D(actualPointOnIntersection, firstPointOnIntersection, expectedIntersectionDirection),
                      epsilon);
      }

      Point3D pointOnPlane1 = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
      Point3D pointOnPlane2 = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

      Vector3D unitVector = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);
      Vector3D planeNormal1 = new Vector3D(unitVector);
      Vector3D planeNormal2 = new Vector3D(unitVector);

      planeNormal1.scale(0.9 * EuclidGeometryTools.ONE_TRILLIONTH);

      assertFalse(EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1, planeNormal1, pointOnPlane2, planeNormal2, 0.5 * Math.PI, new Point3D(),
                                                                     new Vector3D()));
      assertFalse(EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane2, planeNormal2, pointOnPlane1, planeNormal1, 0.5 * Math.PI, new Point3D(),
                                                                     new Vector3D()));

      try
      {
         EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1, planeNormal1, pointOnPlane2, planeNormal2, -Double.MIN_VALUE, new Point3D(),
                                                            new Vector3D());
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1, planeNormal1, pointOnPlane2, planeNormal2, Math.PI / 2.0 + 1.110224e-16,
                                                            new Point3D(), new Vector3D());
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testIsFormingTriangle() throws Exception
   {
      double a, b, c;
      a = 1.0;
      b = 10.0;
      boolean actual = EuclidGeometryTools.isFormingTriangle(b, a, a);
      assertEquals(false, actual);

      a = 1.0;
      actual = EuclidGeometryTools.isFormingTriangle(a, a, a);
      assertEquals(true, actual);

      Random random = new Random(3242L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D point0 = EuclidCoreRandomTools.nextPoint2D(random);
         point0.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D point1 = EuclidCoreRandomTools.nextPoint2D(random);
         point1.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D point2 = EuclidCoreRandomTools.nextPoint2D(random);
         point2.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         double d01 = point0.distance(point1);
         double d12 = point1.distance(point2);
         double d20 = point2.distance(point0);

         a = d01;
         b = d12;
         c = d20;
         assertTrue(EuclidGeometryTools.isFormingTriangle(a, b, c));

         a = d12 + d20 + random.nextDouble();
         b = d12;
         c = d20;
         assertFalse(EuclidGeometryTools.isFormingTriangle(a, b, c));
         a = d01;
         b = d01 + d20 + random.nextDouble();
         c = d20;
         assertFalse(EuclidGeometryTools.isFormingTriangle(a, b, c));
         a = d01;
         b = d12;
         c = d01 + d12 + random.nextDouble();
         assertFalse(EuclidGeometryTools.isFormingTriangle(a, b, c));
      }

      assertFalse(EuclidGeometryTools.isFormingTriangle(0.0, 0.0, 0.0));

      try
      {
         EuclidGeometryTools.isFormingTriangle(-Double.MIN_VALUE, 1.0, 1.0);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
      try
      {
         EuclidGeometryTools.isFormingTriangle(1.0, -Double.MIN_VALUE, 1.0);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
      try
      {
         EuclidGeometryTools.isFormingTriangle(1.0, 1.0, -Double.MIN_VALUE);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testIsPoint2DInsideTriangleABC() throws Exception
   {
      Point2D inside = new Point2D();
      Point2D outside = new Point2D();
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = EuclidCoreRandomTools.nextPoint2D(random);
         a.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D b = EuclidCoreRandomTools.nextPoint2D(random);
         b.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D c = EuclidCoreRandomTools.nextPoint2D(random);
         c.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(a, a, b, c));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(a, c, b, a));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(b, a, b, c));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(b, c, b, a));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(c, a, b, c));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(c, c, b, a));

         inside.interpolate(a, b, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         inside.interpolate(inside, c, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, a, b, c));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, c, b, a));

         outside.interpolate(a, b, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         outside.interpolate(outside, c, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, a, b, c));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         outside.interpolate(outside, c, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, a, b, c));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         outside.interpolate(outside, c, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, a, b, c));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         outside.interpolate(outside, c, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, a, b, c));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, c, b, a));
      }

      Point2D a = new Point2D(1.0, 0.0);
      Point2D b = new Point2D(1.0, 1.0);
      Point2D c = new Point2D(0.0, 1.0);

      // These tests tend to be flaky inside the loop
      inside.interpolate(a, b, 0.5);
      assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, a, b, c));
      assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, c, b, a));
      inside.interpolate(a, c, 0.5);
      assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, a, b, c));
      assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, c, b, a));
      inside.interpolate(b, c, 0.5);
      assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, a, b, c));
      assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, c, b, a));
   }

   @Test
   public void testIsPoint2DOnLine2D() throws Exception
   {
      Random random = new Random(2342334L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine = nextPoint2D(random, 10.0);
         Vector2D lineDirection = nextVector2D(random, -10.0, 10.0);

         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         orthogonal.normalize();

         Point2D query = new Point2D(pointOnLine);
         query.scaleAdd(nextDouble(random, 10.0), lineDirection, pointOnLine);

         assertTrue(EuclidGeometryTools.isPoint2DOnLine2D(query, pointOnLine, lineDirection));

         double shift = nextDouble(random, 1.0) * EuclidGeometryTools.IS_POINT_ON_LINE_EPS;
         query.scaleAdd(shift, orthogonal, query);

         assertTrue(EuclidGeometryTools.isPoint2DOnLine2D(query, pointOnLine, lineDirection));

         query = new Point2D(pointOnLine);
         query.scaleAdd(nextDouble(random, 10.0), lineDirection, pointOnLine);
         shift = nextDouble(random, 1.0, 10.0) * EuclidGeometryTools.IS_POINT_ON_LINE_EPS;
         if (random.nextBoolean())
            shift = -shift;
         query.scaleAdd(shift, orthogonal, query);

         assertFalse(EuclidGeometryTools.isPoint2DOnLine2D(query, pointOnLine, lineDirection));
      }
   }

   @Test
   public void testIsPoint2DOnLineSegment2D() throws Exception
   {
      Random random = new Random(2342334L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with query between endpoints
         Point2D lineSegmentStart = nextPoint2D(random, 10.0);
         Point2D lineSegmentEnd = nextPoint2D(random, 10.0);

         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();

         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineSegmentDirection);

         Point2D queryOnLineSegment = new Point2D();
         queryOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, random.nextDouble());

         assertTrue("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryOnLineSegment, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryOnLineSegment, lineSegmentEnd, lineSegmentStart));

         Point2D queryShifted = new Point2D();
         double shift = nextDouble(random, 1.0) * EuclidGeometryTools.IS_POINT_ON_LINE_EPS;
         queryShifted.scaleAdd(shift, orthogonal, queryOnLineSegment);

         assertTrue("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryShifted, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryShifted, lineSegmentEnd, lineSegmentStart));

         shift = nextDouble(random, 1.0, 10.0) * EuclidGeometryTools.IS_POINT_ON_LINE_EPS;
         if (random.nextBoolean())
            shift = -shift;
         queryShifted.scaleAdd(shift, orthogonal, queryOnLineSegment);

         assertFalse("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryShifted, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryShifted, lineSegmentEnd, lineSegmentStart));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with query outside endpoints, the result depends on the distance between the query and the closest endpoint.
         Point2D lineSegmentStart = nextPoint2D(random, 10.0);
         Point2D lineSegmentEnd = nextPoint2D(random, 10.0);

         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();

         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineSegmentDirection);

         Point2D queryOnLineSegment = new Point2D(lineSegmentEnd);

         assertTrue("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryOnLineSegment, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryOnLineSegment, lineSegmentEnd, lineSegmentStart));

         // Making a shiftVector that shifts the query away by a given distance
         Point2D queryShifted = new Point2D();
         Vector2D shift = new Vector2D();
         if (random.nextBoolean())
            orthogonal.negate();
         shift.interpolate(lineSegmentDirection, orthogonal, random.nextDouble());
         shift.normalize();
         shift.scale(random.nextDouble() * EuclidGeometryTools.IS_POINT_ON_LINE_EPS);
         queryShifted.add(lineSegmentEnd, shift);
         assertTrue("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryShifted, lineSegmentStart, lineSegmentEnd));
         assertTrue("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryShifted, lineSegmentEnd, lineSegmentStart));

         shift.normalize();
         shift.scale(nextDouble(random, 1.0, 10.0) * EuclidGeometryTools.IS_POINT_ON_LINE_EPS);
         queryShifted.add(lineSegmentEnd, shift);
         assertFalse("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryShifted, lineSegmentStart, lineSegmentEnd));
         assertFalse("Iteration: " + i, EuclidGeometryTools.isPoint2DOnLineSegment2D(queryShifted, lineSegmentEnd, lineSegmentStart));
      }
   }

   @Test
   public void testIsPoint2DOnSideOfLine2D() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine = EuclidCoreRandomTools.nextPoint2D(random);
         pointOnLine.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2D(random);
         lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D orthogonalToTheLeft = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         orthogonalToTheLeft.normalize();
         Vector2D orthogonalToTheRight = new Vector2D();
         orthogonalToTheRight.setAndNegate(orthogonalToTheLeft);

         Point2D secondPointOnLine = new Point2D();
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointOnLine);

         Point2D query = new Point2D();
         double alpha = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         query.scaleAdd(alpha, orthogonalToTheLeft, secondPointOnLine);
         assertTrue(EuclidGeometryTools.isPoint2DOnSideOfLine2D(query, pointOnLine, lineDirection, true));
         assertFalse(EuclidGeometryTools.isPoint2DOnSideOfLine2D(query, pointOnLine, lineDirection, false));
         query.scaleAdd(alpha, orthogonalToTheRight, secondPointOnLine);
         assertFalse(EuclidGeometryTools.isPoint2DOnSideOfLine2D(query, pointOnLine, lineDirection, true));
         assertTrue(EuclidGeometryTools.isPoint2DOnSideOfLine2D(query, pointOnLine, lineDirection, false));
         query.set(pointOnLine);
         assertFalse(EuclidGeometryTools.isPoint2DOnSideOfLine2D(query, pointOnLine, lineDirection, true));
         assertFalse(EuclidGeometryTools.isPoint2DOnSideOfLine2D(query, pointOnLine, lineDirection, false));
      }
   }

   @Test
   public void testNormal3DFromThreePoint3Ds() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D expectedPlaneNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0);

         Point3D firstPointOnPlane = EuclidCoreRandomTools.nextPoint3D(random);
         firstPointOnPlane.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D secondPointOnPlane = new Point3D();
         Point3D thirdPointOnPlane = new Point3D();

         Vector3D secondOrthogonalToNormal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, expectedPlaneNormal, true);
         Vector3D thirdOrthogonalToNormal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, expectedPlaneNormal, true);

         secondPointOnPlane.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0), secondOrthogonalToNormal, firstPointOnPlane);
         thirdPointOnPlane.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0), thirdOrthogonalToNormal, firstPointOnPlane);

         Vector3D actualPlaneNormal = EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);

         if (expectedPlaneNormal.dot(actualPlaneNormal) < 0.0)
            actualPlaneNormal.negate();

         EuclidCoreTestTools.assertTuple3DEquals(expectedPlaneNormal, actualPlaneNormal, EPSILON);

         assertNull(EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, firstPointOnPlane));
      }
   }

   @Test
   public void testOrthogonalProjectionOnLine2D() throws Exception
   {
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = EuclidCoreRandomTools.nextPoint2D(random);
         firstPointOnLine.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D secondPointOnLine = EuclidCoreRandomTools.nextPoint2D(random);
         secondPointOnLine.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(secondPointOnLine, firstPointOnLine);
         lineDirection.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));

         Point2D expectionProjection = new Point2D();
         expectionProjection.interpolate(firstPointOnLine, secondPointOnLine, EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D perpendicularToLineDirection = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         perpendicularToLineDirection.normalize();

         Point2D testPoint = new Point2D();
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), perpendicularToLineDirection, expectionProjection);

         Point2D actualProjection = new Point2D();
         boolean success;

         success = EuclidGeometryTools.orthogonalProjectionOnLine2D(testPoint, firstPointOnLine, secondPointOnLine, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectionProjection, actualProjection, EPSILON);

         success = EuclidGeometryTools.orthogonalProjectionOnLine2D(testPoint, firstPointOnLine, lineDirection, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectionProjection, actualProjection, EPSILON);

         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLine2D(testPoint, firstPointOnLine, secondPointOnLine);
         EuclidCoreTestTools.assertTuple2DEquals(expectionProjection, actualProjection, EPSILON);

         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLine2D(testPoint, firstPointOnLine, lineDirection);
         EuclidCoreTestTools.assertTuple2DEquals(expectionProjection, actualProjection, EPSILON);

         lineDirection.normalize();
         lineDirection.scale(0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         assertNull(EuclidGeometryTools.orthogonalProjectionOnLine2D(testPoint, firstPointOnLine, lineDirection));
         assertFalse(EuclidGeometryTools.orthogonalProjectionOnLine2D(testPoint, firstPointOnLine, lineDirection, new Point2D()));

         secondPointOnLine.add(firstPointOnLine, lineDirection);
         assertNull(EuclidGeometryTools.orthogonalProjectionOnLine2D(testPoint, firstPointOnLine, secondPointOnLine));
         assertFalse(EuclidGeometryTools.orthogonalProjectionOnLine2D(testPoint, firstPointOnLine, secondPointOnLine, new Point2D()));
      }
   }

   @Test
   public void testOrthogonalProjectionOnLine3D() throws Exception
   {
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnLine = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnLine.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D lineDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         Point3D expectedProjection = new Point3D();
         expectedProjection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, pointOnLine);
         Vector3D perpendicularToLineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineDirection, true);

         Point3D testPoint = new Point3D();
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), perpendicularToLineDirection, expectedProjection);

         Point3D actualProjection = new Point3D();
         boolean success;

         success = EuclidGeometryTools.orthogonalProjectionOnLine3D(testPoint, pointOnLine, lineDirection, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);

         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLine3D(testPoint, pointOnLine, lineDirection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);

         lineDirection.normalize();
         lineDirection.scale(0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         assertFalse(EuclidGeometryTools.orthogonalProjectionOnLine3D(testPoint, pointOnLine, lineDirection, actualProjection));
         assertNull(EuclidGeometryTools.orthogonalProjectionOnLine3D(testPoint, pointOnLine, lineDirection));
      }
   }

   @Test
   public void testOrthogonalProjectionOnLineSegment2D() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         EuclidGeometryTools.perpendicularVector2D(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D expectedProjection = new Point2D();
         Point2D testPoint = new Point2D();
         Point2D actualProjection = new Point2D();
         boolean success;

         // Between end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, expectedProjection);
         success = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);

         // Before end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentStart);
         success = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);

         // After end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentEnd);
         success = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);

         // The line segment is very small
         testPoint = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         expectedProjection.set(lineSegmentStart);
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         lineSegmentDirection.scale(0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         lineSegmentEnd.add(lineSegmentStart, lineSegmentDirection);
         success = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);
      }
   }

   @Test
   public void testOrthogonalProjectionOnLineSegment3D() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D lineSegmentEnd = EuclidCoreRandomTools.nextPoint3D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         Vector3D orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection, true);
         Point3D expectedProjection = new Point3D();
         Point3D testPoint = new Point3D();
         Point3D actualProjection = new Point3D();
         boolean success;

         // Between end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, expectedProjection);
         success = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);

         // Before end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentStart);
         success = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);

         // After end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentEnd);
         success = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);

         // The line segment is very small
         testPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         expectedProjection.set(lineSegmentStart);
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         lineSegmentDirection.scale(0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         lineSegmentEnd.add(lineSegmentStart, lineSegmentDirection);
         success = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }
   }

   @Test
   public void testOrthogonalProjectionOnPlane3D() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.nextPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = EuclidCoreRandomTools.nextOrthogonalVector3D(random, planeNormal, true);
         Point3D expectedProjection = new Point3D();
         expectedProjection.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Point3D pointToProject = new Point3D();
         double distanceOffPlane = EuclidCoreRandomTools.nextDouble(random, 10.0);
         pointToProject.scaleAdd(distanceOffPlane, planeNormal, expectedProjection);

         Point3D actualProjection = new Point3D();
         boolean success;

         success = EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal, actualProjection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);

         // Test failure case.
         planeNormal.normalize();
         planeNormal.scale(0.9 * EuclidGeometryTools.ONE_TRILLIONTH);

         assertFalse(EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal, actualProjection));
         assertNull(EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal));
      }
   }

   @Test
   public void testPercentageAlongLineSegment2D() throws Exception
   {
      Random random = new Random(23424L);

      // Test on line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         Point2D pointOnLineSegment = new Point2D();

         // Test between end points
         double expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         double actualPercentage = EuclidGeometryTools.percentageAlongLineSegment2D(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);

         // Test before end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = EuclidGeometryTools.percentageAlongLineSegment2D(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);

         // Test after end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = EuclidGeometryTools.percentageAlongLineSegment2D(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);
      }

      // Test off line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         Point2D pointOffLineSegment = new Point2D();
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineSegmentDirection);
         orthogonal.normalize();

         // Test between end points
         double expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         double actualPercentage = EuclidGeometryTools.percentageAlongLineSegment2D(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);

         // Test before end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = EuclidGeometryTools.percentageAlongLineSegment2D(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);

         // Test after end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = EuclidGeometryTools.percentageAlongLineSegment2D(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);
      }

      // The line segment is very small
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D lineSegmentDirection = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, EuclidGeometryTools.ONE_TRILLIONTH);
         Point2D lineSegmentEnd = new Point2D();
         lineSegmentEnd.add(lineSegmentStart, lineSegmentDirection);

         double expectedPercentage = 0.0;
         Point2D testPoint = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         double actualPercentage = EuclidGeometryTools.percentageAlongLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertTrue(expectedPercentage == actualPercentage);
      }
   }

   @Test
   public void testPercentageAlongLineSegment3D() throws Exception
   {
      Random random = new Random(23424L);

      // Test on line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Point3D lineSegmentEnd = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

         Point3D pointOnLineSegment = new Point3D();

         // Test between end points
         double expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         double actualPercentage = EuclidGeometryTools.percentageAlongLineSegment3D(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);

         // Test before end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = EuclidGeometryTools.percentageAlongLineSegment3D(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);

         // Test after end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         pointOnLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         actualPercentage = EuclidGeometryTools.percentageAlongLineSegment3D(pointOnLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);
      }

      // Test off line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = EuclidCoreRandomTools.nextPoint3D(random, -10.0, 10.0);
         Point3D lineSegmentEnd = EuclidCoreRandomTools.nextPoint3D(random, -10.0, 10.0);

         Point3D pointOffLineSegment = new Point3D();
         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         Vector3D orthogonal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, lineSegmentDirection, true);

         // Test between end points
         double expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         double actualPercentage = EuclidGeometryTools.percentageAlongLineSegment3D(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);

         // Test before end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, -10.0, 0.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = EuclidGeometryTools.percentageAlongLineSegment3D(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);

         // Test after end points
         expectedPercentage = EuclidCoreRandomTools.nextDouble(random, 1.0, 10.0);
         pointOffLineSegment.interpolate(lineSegmentStart, lineSegmentEnd, expectedPercentage);
         pointOffLineSegment.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), orthogonal, pointOffLineSegment);
         actualPercentage = EuclidGeometryTools.percentageAlongLineSegment3D(pointOffLineSegment, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedPercentage, actualPercentage, EPSILON);
      }

      // The line segment is very small
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = EuclidCoreRandomTools.nextPoint3D(random, 10.0);
         Vector3D lineSegmentDirection = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidGeometryTools.ONE_TRILLIONTH);
         Point3D lineSegmentEnd = new Point3D();
         lineSegmentEnd.add(lineSegmentStart, lineSegmentDirection);

         double expectedPercentage = 0.0;
         Point3D testPoint = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

         double actualPercentage = EuclidGeometryTools.percentageAlongLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertTrue(expectedPercentage == actualPercentage);
      }
   }

   @Test
   public void testPerpendicularBisector2D() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         Point2D expectedBisectorStart = new Point2D();
         expectedBisectorStart.interpolate(lineSegmentStart, lineSegmentEnd, 0.5);
         Vector2D expectedBisectorDirection = new Vector2D();
         expectedBisectorDirection.sub(lineSegmentEnd, lineSegmentStart);
         EuclidGeometryTools.perpendicularVector2D(expectedBisectorDirection, expectedBisectorDirection);
         expectedBisectorDirection.normalize();

         Point2D actualBisectorStart = new Point2D();
         Vector2D actualBisectorDirection = new Vector2D();
         boolean success;
         success = EuclidGeometryTools.perpendicularBisector2D(lineSegmentStart, lineSegmentEnd, actualBisectorStart, actualBisectorDirection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedBisectorStart, actualBisectorStart, EPSILON);
         EuclidCoreTestTools.assertTuple2DEquals(expectedBisectorDirection, actualBisectorDirection, EPSILON);

         Point2D pointOnBisector = new Point2D();
         pointOnBisector.scaleAdd(1.0, actualBisectorDirection, actualBisectorStart);
         assertTrue(EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(pointOnBisector, lineSegmentStart, lineSegmentEnd));

         // Test with small segment
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         lineSegmentDirection.scale(0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         lineSegmentEnd.add(lineSegmentStart, lineSegmentDirection);
         assertFalse(EuclidGeometryTools.perpendicularBisector2D(lineSegmentStart, lineSegmentEnd, actualBisectorStart, actualBisectorDirection));
      }
   }

   @Test
   public void testPerpendicularBisectorSegment2D() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D lineSegmentEnd = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D lineSegmentMidpoint = new Point2D();
         lineSegmentMidpoint.interpolate(lineSegmentStart, lineSegmentEnd, 0.5);
         Vector2D lineSegmentDirection = new Vector2D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);

         double bisectorSegmentHalfLength = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);
         Point2D bisectorSegmentStart = new Point2D();
         Point2D bisectorSegmentEnd = new Point2D();
         Vector2D bisectorDirection = new Vector2D();

         boolean success = EuclidGeometryTools.perpendicularBisectorSegment2D(lineSegmentStart, lineSegmentEnd, bisectorSegmentHalfLength, bisectorSegmentStart,
                                                                              bisectorSegmentEnd);
         assertTrue(success);

         bisectorDirection.sub(bisectorSegmentEnd, bisectorSegmentStart);
         assertEquals(2.0 * bisectorSegmentHalfLength, bisectorDirection.length(), EPSILON);
         assertEquals(0.0, lineSegmentDirection.dot(bisectorDirection), EPSILON);
         assertEquals(bisectorSegmentHalfLength, EuclidGeometryTools.distanceFromPoint2DToLine2D(bisectorSegmentStart, lineSegmentStart, lineSegmentEnd),
                      EPSILON);
         assertEquals(bisectorSegmentHalfLength, EuclidGeometryTools.distanceFromPoint2DToLine2D(bisectorSegmentEnd, lineSegmentStart, lineSegmentEnd),
                      EPSILON);
         assertTrue(EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(bisectorSegmentStart, lineSegmentStart, lineSegmentEnd));
         assertTrue(EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(bisectorSegmentEnd, lineSegmentStart, lineSegmentEnd));
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentMidpoint, EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(bisectorSegmentStart,
                                                                                                                              lineSegmentStart, lineSegmentEnd),
                                                 EPSILON);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentMidpoint,
                                                 EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(bisectorSegmentEnd, lineSegmentStart, lineSegmentEnd),
                                                 EPSILON);

         List<Point2D> bisectorSegmentEndpoints = EuclidGeometryTools.perpendicularBisectorSegment2D(lineSegmentStart, lineSegmentEnd,
                                                                                                     bisectorSegmentHalfLength);
         bisectorSegmentStart = bisectorSegmentEndpoints.get(0);
         bisectorSegmentEnd = bisectorSegmentEndpoints.get(1);

         bisectorDirection.sub(bisectorSegmentEnd, bisectorSegmentStart);
         assertEquals(2.0 * bisectorSegmentHalfLength, bisectorDirection.length(), EPSILON);
         assertEquals(0.0, lineSegmentDirection.dot(bisectorDirection), EPSILON);
         assertEquals(bisectorSegmentHalfLength, EuclidGeometryTools.distanceFromPoint2DToLine2D(bisectorSegmentStart, lineSegmentStart, lineSegmentEnd),
                      EPSILON);
         assertEquals(bisectorSegmentHalfLength, EuclidGeometryTools.distanceFromPoint2DToLine2D(bisectorSegmentEnd, lineSegmentStart, lineSegmentEnd),
                      EPSILON);
         assertTrue(EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(bisectorSegmentStart, lineSegmentStart, lineSegmentEnd));
         assertTrue(EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(bisectorSegmentEnd, lineSegmentStart, lineSegmentEnd));
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentMidpoint, EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(bisectorSegmentStart,
                                                                                                                              lineSegmentStart, lineSegmentEnd),
                                                 EPSILON);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentMidpoint,
                                                 EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(bisectorSegmentEnd, lineSegmentStart, lineSegmentEnd),
                                                 EPSILON);

         // Test with small segment
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         lineSegmentDirection.scale(0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         lineSegmentEnd.add(lineSegmentStart, lineSegmentDirection);
         assertNull(EuclidGeometryTools.perpendicularBisectorSegment2D(lineSegmentStart, lineSegmentEnd, bisectorSegmentHalfLength));
         assertFalse(EuclidGeometryTools.perpendicularBisectorSegment2D(lineSegmentStart, lineSegmentEnd, bisectorSegmentHalfLength, bisectorSegmentStart,
                                                                        bisectorSegmentEnd));
      }
   }

   @Test
   public void testPerpedicularVector2D() throws Exception
   {
      Vector2D vector = new Vector2D(15.0, 10.0);
      Vector2D expectedReturn = new Vector2D(-10.0, 15.0);
      Vector2D actualReturn = EuclidGeometryTools.perpendicularVector2D(vector);
      assertEquals("return value", expectedReturn, actualReturn);
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         vector = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         Vector2D perpendicularVector = EuclidGeometryTools.perpendicularVector2D(vector);
         assertEquals(vector.length(), perpendicularVector.length(), EPSILON);
         assertEquals(vector.length() * vector.length(), vector.cross(perpendicularVector), EPSILON);
         assertEquals(0.0, vector.dot(perpendicularVector), EPSILON);
         assertEquals(Math.PI / 2.0, vector.angle(perpendicularVector), EPSILON);
      }
   }

   @Test
   public void testPerpendicularVector3DFromLine3DToPoint3D() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D expectedPerpendicularVector = EuclidCoreRandomTools.nextVector3D(random);
         expectedPerpendicularVector.scale(EuclidCoreRandomTools.nextDouble(random, 10.0));
         Point3D expectedIntersection = EuclidCoreRandomTools.nextPoint3D(random, 10.0);

         Vector3D lineDirection = EuclidCoreRandomTools.nextOrthogonalVector3D(random, expectedPerpendicularVector, true);
         Point3D firstPointOnLine = new Point3D();
         firstPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, expectedIntersection);
         Point3D secondPointOnLine = new Point3D();
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, expectedIntersection);

         Point3D point = new Point3D();
         point.add(expectedIntersection, expectedPerpendicularVector);

         Point3D actualIntersection = new Point3D();
         double epsilon = EPSILON;

         if (firstPointOnLine.distance(secondPointOnLine) < 5.0e-4)
            epsilon = 1.0e-10; // Loss of precision when the given points defining the line are getting close.

         Vector3D actualPerpendicularVector = EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point, firstPointOnLine, secondPointOnLine,
                                                                                                           actualIntersection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, epsilon);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPerpendicularVector, actualPerpendicularVector, epsilon);

         actualPerpendicularVector = EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point, firstPointOnLine, secondPointOnLine, null);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPerpendicularVector, actualPerpendicularVector, epsilon);

         // Test failure case
         lineDirection.normalize();
         lineDirection.scale(0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         secondPointOnLine.add(firstPointOnLine, lineDirection);
         assertNull(EuclidGeometryTools.perpendicularVector3DFromLine3DToPoint3D(point, firstPointOnLine, secondPointOnLine, actualIntersection));
      }
   }

   @Test
   public void testPythagorasGetCathetus() throws Exception
   {
      try
      {
         EuclidGeometryTools.pythagorasGetCathetus(-Double.MIN_VALUE, 1.0);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         EuclidGeometryTools.pythagorasGetCathetus(1.0, -Double.MIN_VALUE);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         EuclidGeometryTools.pythagorasGetCathetus(1.0, 2.0);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      EuclidGeometryTools.pythagorasGetCathetus(1.0, 1.0);

      Random random = new Random(23434L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = new Point2D();
         Point2D b = new Point2D(EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0), 0.0);
         Point2D c = new Point2D(0.0, EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0));

         double hypotenuseC = b.distance(c);
         double cathetusA = a.distance(b);
         double expectedCathetusB = a.distance(c);
         double actualCathetusB = EuclidGeometryTools.pythagorasGetCathetus(hypotenuseC, cathetusA);
         assertEquals(expectedCathetusB, actualCathetusB, EPSILON);
      }
   }

   @Test
   public void testPythagorasGetHypotenuse() throws Exception
   {
      try
      {
         EuclidGeometryTools.pythagorasGetHypotenuse(-Double.MIN_VALUE, 1.0);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         EuclidGeometryTools.pythagorasGetHypotenuse(1.0, -Double.MIN_VALUE);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      Random random = new Random(23434L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = new Point2D();
         Point2D b = new Point2D(EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0), 0.0);
         Point2D c = new Point2D(0.0, EuclidCoreRandomTools.nextDouble(random, 0.001, 10.0));

         double cathetusA = a.distance(b);
         double cathetusB = a.distance(c);
         double expectedHypotenuseC = b.distance(c);
         double actualHypotenuseC = EuclidGeometryTools.pythagorasGetHypotenuse(cathetusA, cathetusB);
         assertEquals(expectedHypotenuseC, actualHypotenuseC, EPSILON);
      }
   }

   @Test
   public void testRadiusOfArc() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         double expectedArcRadius = EuclidCoreRandomTools.nextDouble(random, 0.1, 100.0);
         double chordAngle = EuclidCoreRandomTools.nextDouble(random, -3.0 * Math.PI, 3.0 * Math.PI);
         double chordLength = 2.0 * expectedArcRadius * Math.sin(0.5 * chordAngle);
         double actualArcRadius = EuclidGeometryTools.radiusOfArc(chordLength, chordAngle);
         assertEquals(expectedArcRadius, actualArcRadius, EPSILON);
      }

      assertTrue(Double.isNaN(EuclidGeometryTools.radiusOfArc(1.0, 0.0)));
      assertTrue(Double.isNaN(EuclidGeometryTools.radiusOfArc(1.0, Math.PI)));
      assertTrue(Double.isNaN(EuclidGeometryTools.radiusOfArc(1.0, -Math.PI)));
   }

   @Test
   public void testSignedDistanceFromPoint2DToLine2D() throws Exception
   {
      Random random = new Random(243234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Vector2D lineDirection = EuclidCoreRandomTools.nextVector2D(random, -10.0, 10.0);
         Point2D secondPointOnLine = new Point2D();
         secondPointOnLine.add(firstPointOnLine, lineDirection);

         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         orthogonal.normalize();
         boolean expectingNegativeSign = random.nextBoolean();

         if (expectingNegativeSign)
            orthogonal.negate();

         double expectedDistance = EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0);

         Point2D query = new Point2D();
         query.scaleAdd(expectedDistance, orthogonal, firstPointOnLine);
         query.scaleAdd(EuclidCoreRandomTools.nextDouble(random, 10.0), lineDirection, query);

         double actualDistance = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(query, firstPointOnLine, lineDirection);
         assertEquals("Iteration: " + i, expectedDistance, expectingNegativeSign ? -actualDistance : actualDistance, EPSILON);
         actualDistance = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(query, firstPointOnLine, secondPointOnLine);
         assertEquals("Iteration: " + i, expectedDistance, expectingNegativeSign ? -actualDistance : actualDistance, EPSILON);

         // Test with a line direction that a magnitude that is too small
         lineDirection.normalize();
         lineDirection.scale(0.999 * EuclidGeometryTools.ONE_TRILLIONTH);
         secondPointOnLine.add(firstPointOnLine, lineDirection);
         expectedDistance = firstPointOnLine.distance(query);
         actualDistance = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(query, firstPointOnLine, lineDirection);
         assertEquals("Iteration: " + i, expectedDistance, actualDistance, EPSILON);
         actualDistance = EuclidGeometryTools.signedDistanceFromPoint2DToLine2D(query, firstPointOnLine, secondPointOnLine);
         assertEquals("Iteration: " + i, expectedDistance, actualDistance, EPSILON);
      }
   }

   @Test
   public void testTopVertex3DOfIsoscelesTriangle3D() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D expectedB = EuclidCoreRandomTools.nextPoint3D(random, -10.0, 10.0);
         Point3D a = EuclidCoreRandomTools.nextPoint3D(random, -10.0, 10.0);
         Vector3D ba = new Vector3D();
         ba.sub(a, expectedB);

         double abcAngle = EuclidCoreRandomTools.nextDouble(random, 0.0, Math.PI / 2.0);

         Vector3D triangleNormal = EuclidCoreRandomTools.nextOrthogonalVector3D(random, ba, true);
         triangleNormal.scale(EuclidCoreRandomTools.nextDouble(random, 0.0, 10.0));
         AxisAngle abcAxisAngle = new AxisAngle(triangleNormal, abcAngle);
         RotationMatrix abcRotationMatrix = new RotationMatrix();
         abcRotationMatrix.set(abcAxisAngle);
         Vector3D bc = new Vector3D();
         abcRotationMatrix.transform(ba, bc);

         Point3D c = new Point3D();
         c.add(bc, expectedB);

         double epsilon = 1.0e-11;

         Point3D actualB = new Point3D();
         EuclidGeometryTools.topVertex3DOfIsoscelesTriangle3D(a, c, triangleNormal, abcAngle, actualB);
         EuclidCoreTestTools.assertTuple3DEquals(expectedB, actualB, epsilon);
         assertEquals(abcAngle, ba.angle(bc), epsilon);
      }
   }

   @Test
   public void testTriangleBisector2D() throws Exception
   {
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D b = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D c = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         Vector2D ba = new Vector2D();
         ba.sub(a, b);
         Vector2D bc = new Vector2D();
         bc.sub(c, b);

         double abcAngle = ba.angle(bc);
         Point2D x = new Point2D();
         Vector2D bx = new Vector2D();

         boolean success = EuclidGeometryTools.triangleBisector2D(a, b, c, x);
         assertTrue(success);
         bx.sub(x, b);
         double abxAngle = ba.angle(bx);
         assertEquals(0.5 * abcAngle, abxAngle, EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(x, a, c), EPSILON);

         x = EuclidGeometryTools.triangleBisector2D(a, b, c);
         bx.sub(x, b);
         abxAngle = ba.angle(bx);
         assertEquals(0.5 * abcAngle, abxAngle, EPSILON);
         assertEquals(0.0, EuclidGeometryTools.distanceFromPoint2DToLine2D(x, a, c), EPSILON);
      }

      // Test failure case 1
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D small = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         Point2D a = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D b = new Point2D();
         b.add(a, small);
         Point2D c = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         assertFalse(EuclidGeometryTools.triangleBisector2D(a, b, c, new Point2D()));
         assertNull(EuclidGeometryTools.triangleBisector2D(a, b, c));
      }

      // Test failure case 2
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D small = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         Point2D a = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D b = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D c = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         c.add(b, small);

         assertFalse(EuclidGeometryTools.triangleBisector2D(a, b, c, new Point2D()));
         assertNull(EuclidGeometryTools.triangleBisector2D(a, b, c));
      }

      // Test failure case 3
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D small = EuclidCoreRandomTools.nextVector2DWithFixedLength(random, 0.9 * EuclidGeometryTools.ONE_TRILLIONTH);
         Point2D a = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D b = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D c = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         c.add(a, small);

         assertFalse(EuclidGeometryTools.triangleBisector2D(a, b, c, new Point2D()));
         assertNull(EuclidGeometryTools.triangleBisector2D(a, b, c));
      }
   }

   @Test
   public void testUnknownTriangleAngleByLawOfCosine() throws Exception
   {
      Random random = new Random(34534L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D b = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D c = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         Vector2D ab = new Vector2D();
         ab.sub(b, a);
         Vector2D ba = new Vector2D();
         ba.sub(a, b);
         Vector2D ac = new Vector2D();
         ac.sub(c, a);
         Vector2D ca = new Vector2D();
         ca.sub(a, c);
         Vector2D bc = new Vector2D();
         bc.sub(c, b);
         Vector2D cb = new Vector2D();
         cb.sub(b, c);

         // The three edge lengths
         double abLength = ab.length();
         double acLength = ac.length();
         double bcLength = bc.length();

         // The three angles
         double abc = Math.abs(ba.angle(bc));
         double bca = Math.abs(cb.angle(ca));
         double cab = Math.abs(ac.angle(ab));

         assertEquals(cab, EuclidGeometryTools.unknownTriangleAngleByLawOfCosine(abLength, acLength, bcLength), EPSILON);
         assertEquals(bca, EuclidGeometryTools.unknownTriangleAngleByLawOfCosine(acLength, bcLength, abLength), EPSILON);
         assertEquals(abc, EuclidGeometryTools.unknownTriangleAngleByLawOfCosine(abLength, bcLength, acLength), EPSILON);
      }
   }

   @Test
   public void testUnknownTriangleSideLengthByLawOfCosine() throws Exception
   {
      Random random = new Random(34534L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D b = EuclidCoreRandomTools.nextPoint2D(random, 10.0);
         Point2D c = EuclidCoreRandomTools.nextPoint2D(random, 10.0);

         Vector2D ab = new Vector2D();
         ab.sub(b, a);
         Vector2D ba = new Vector2D();
         ba.sub(a, b);
         Vector2D ac = new Vector2D();
         ac.sub(c, a);
         Vector2D ca = new Vector2D();
         ca.sub(a, c);
         Vector2D bc = new Vector2D();
         bc.sub(c, b);
         Vector2D cb = new Vector2D();
         cb.sub(b, c);

         // The three edge lengths
         double abLength = ab.length();
         double acLength = ac.length();
         double bcLength = bc.length();

         // The three angles
         double abc = Math.abs(ba.angle(bc));
         double bca = Math.abs(cb.angle(ca));
         double cab = Math.abs(ac.angle(ab));

         assertEquals(bcLength, EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(abLength, acLength, cab), EPSILON);
         assertEquals(abLength, EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(acLength, bcLength, bca), EPSILON);
         assertEquals(acLength, EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(abLength, bcLength, abc), EPSILON);
      }

      try
      {
         EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(-Double.MIN_VALUE, 1.0, 1.0);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(1.0, -Double.MIN_VALUE, 1.0);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         EuclidGeometryTools.unknownTriangleSideLengthByLawOfCosine(1.0, 1.0, Math.PI + 2.22045e-16);
         fail("Should have thrown a " + RuntimeException.class.getSimpleName());
      }
      catch (RuntimeException e)
      {
         // good
      }
   }
}
