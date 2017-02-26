package us.ihmc.euclid.geometry;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidGeometryToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testAngleFromFirstToSecondVector2D() throws Exception
   {
      Random random = new Random(4353L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D firstVector = EuclidCoreRandomTools.generateRandomVector2D(random);
         Vector2D secondVector = EuclidCoreRandomTools.generateRandomVector2D(random);

         firstVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         secondVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         double expectedAngle = firstVector.angle(secondVector);
         double actualAngle = EuclidGeometryTools.angleFromFirstToSecondVector2D(firstVector.getX(), firstVector.getY(), secondVector.getX(),
                                                                                 secondVector.getY());

         EuclidCoreTestTools.assertAngleEquals(expectedAngle, actualAngle, EPSILON);
      }
   }

   @Test
   public void testAngleFromFirstToSecondVector3D() throws Exception
   {
      Random random = new Random(4353L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         Vector3D secondVector = EuclidCoreRandomTools.generateRandomVector3D(random);

         firstVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         secondVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

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
         Vector2D lineDirection1 = EuclidCoreRandomTools.generateRandomVector2D(random);
         lineDirection1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         double angleEpsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);

         Vector2D lineDirection2 = new Vector2D();
         EuclidGeometryTools.rotateTuple2D(rotationAngle, lineDirection1, lineDirection2);
         lineDirection2.normalize();
         lineDirection2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

         Point2D firstPointOnLine1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D secondPointOnLine1 = new Point2D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection1, firstPointOnLine1);

         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection1);
         orthogonal.normalize();
         double distance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         double distanceEspilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);

         Point2D firstPointOnLine2 = new Point2D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point2D secondPointOnLine2 = new Point2D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, firstPointOnLine2);

         boolean expectedCollinear = rotationAngle < angleEpsilon && distance < distanceEspilon;
         boolean actualCollinear = EuclidGeometryTools.areLine2DsCollinear(firstPointOnLine1, secondPointOnLine1, firstPointOnLine2, secondPointOnLine2,
                                                                           angleEpsilon, distanceEspilon);
         assertEquals(expectedCollinear, actualCollinear);
      }

      // Test only the distance with parallel line segments.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D lineDirection = EuclidCoreRandomTools.generateRandomVector2D(random);
         lineDirection.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         orthogonal.normalize();

         double angleEpsilon = EuclidGeometryTools.ONE_MILLIONTH;

         Point2D firstPointOnLine1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D secondPointOnLine1 = new Point2D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, firstPointOnLine1);

         double distance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         double distanceEspilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);

         Point2D firstPointOnLine2 = new Point2D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point2D secondPointOnLine2 = new Point2D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, firstPointOnLine2);
         firstPointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, firstPointOnLine2);

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
         Vector3D lineDirection1 = EuclidCoreRandomTools.generateRandomRotationVector(random);
         lineDirection1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         double angleEpsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);
         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineDirection1, true);
         AxisAngle axisAngle = new AxisAngle(orthogonal, rotationAngle);

         Vector3D lineDirection2 = new Vector3D();
         axisAngle.transform(lineDirection1, lineDirection2);
         lineDirection2.normalize();
         lineDirection2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

         Point3D firstPointOnLine1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D secondPointOnLine1 = new Point3D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection1, firstPointOnLine1);

         double distance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         double distanceEspilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);

         Point3D firstPointOnLine2 = new Point3D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point3D secondPointOnLine2 = new Point3D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, firstPointOnLine2);

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
         Vector3D lineDirection = EuclidCoreRandomTools.generateRandomVector3D(random);
         lineDirection.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineDirection, true);

         double angleEpsilon = EuclidGeometryTools.ONE_MILLIONTH;

         Point3D firstPointOnLine1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         firstPointOnLine1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D secondPointOnLine1 = new Point3D();
         secondPointOnLine1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, firstPointOnLine1);

         double distance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         double distanceEspilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);

         Point3D firstPointOnLine2 = new Point3D();
         firstPointOnLine2.scaleAdd(distance, orthogonal, firstPointOnLine1);
         Point3D secondPointOnLine2 = new Point3D();
         secondPointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, firstPointOnLine2);
         firstPointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, firstPointOnLine2);

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
         Point3D pointOnPlane1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnPlane1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D planeNormal1 = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);

         Point3D pointOnPlane2 = new Point3D();
         Vector3D planeNormal2 = new Vector3D();

         double distanceEpsilon = EuclidCoreRandomTools.generateRandomDouble(random, 1.0);
         double distanceBetweenPlanes = EuclidCoreRandomTools.generateRandomDouble(random, 1.0);

         pointOnPlane2.scaleAdd(distanceBetweenPlanes, planeNormal1, pointOnPlane1);

         Vector3D rotationAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, planeNormal1, true);
         double angleEpsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);

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
   public void testAreVector2DsCollinear() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D firstVector = EuclidCoreRandomTools.generateRandomVector2D(random);
         firstVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         double angleEpsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);

         Vector2D secondVector = new Vector2D();
         EuclidGeometryTools.rotateTuple2D(rotationAngle, firstVector, secondVector);
         secondVector.normalize();
         secondVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

         assertEquals(rotationAngle < angleEpsilon, EuclidGeometryTools.areVector2DsCollinear(firstVector, secondVector, angleEpsilon));
      }

      // Try again with small values
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector2D firstVector = EuclidCoreRandomTools.generateRandomVector2D(random);
         firstVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         double angleEpsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, EuclidGeometryTools.ONE_MILLIONTH * Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, EuclidGeometryTools.ONE_MILLIONTH * Math.PI / 2.0);
         if (Math.abs(rotationAngle - angleEpsilon) < 1.0e-7)
            continue; // This is the limit of accuracy.

         Vector2D secondVector = new Vector2D();
         EuclidGeometryTools.rotateTuple2D(rotationAngle, firstVector, secondVector);

         assertEquals(rotationAngle < angleEpsilon, EuclidGeometryTools.areVector2DsCollinear(firstVector, secondVector, angleEpsilon));
      }
   }

   @Test
   public void testAreVector3DsCollinear() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         firstVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector3D rotationAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, firstVector, true);
         double angleEpsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI / 2.0);

         AxisAngle rotationAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.normalize();
         secondVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

         assertEquals(rotationAngle < angleEpsilon, EuclidGeometryTools.areVector3DsCollinear(firstVector, secondVector, angleEpsilon));
      }

      // Try again with small values
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         firstVector.normalize();

         Vector3D rotationAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, firstVector, true);
         double angleEpsilon = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, EuclidGeometryTools.ONE_MILLIONTH * Math.PI / 2.0);
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, EuclidGeometryTools.ONE_MILLIONTH * Math.PI / 2.0);
         if (Math.abs(rotationAngle - angleEpsilon) < 1.0e-7)
            continue; // This is the limit of accuracy.

         AxisAngle rotationAxisAngle = new AxisAngle(rotationAxis, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);

         assertEquals(rotationAngle < angleEpsilon, EuclidGeometryTools.areVector3DsCollinear(firstVector, secondVector, angleEpsilon));
      }
   }

   @Test
   public void testAveragePoint2Ds() throws Exception
   {
      ArrayList<Point2D> points = new ArrayList<Point2D>();
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

      ArrayList<Point2D> points1 = new ArrayList<Point2D>();
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

      ArrayList<Point2D> points2 = new ArrayList<Point2D>();
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
      ArrayList<Point3D> points = new ArrayList<Point3D>();
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

      ArrayList<Point3D> points1 = new ArrayList<Point3D>();
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
         Vector3D firstVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         firstVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         double expectedAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI);
         Vector3D expectedAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(expectedAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

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
         Vector3D firstVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         firstVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         double expectedAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         Vector3D expectedAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(expectedAxisAngle);

         Vector3D secondVector = new Vector3D();
         rotationMatrix.transform(firstVector, secondVector);
         secondVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

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
         Vector3D firstVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         firstVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
         double expectedAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.00001, 0.001);
         if (random.nextBoolean())
            expectedAngle = -expectedAngle;
         expectedAngle += Math.PI;
         Vector3D expectedAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, firstVector, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         Vector3D secondVector = new Vector3D();
         expectedAxisAngle.transform(firstVector, secondVector);
         secondVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));

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
         EuclidCoreTestTools.assertAxisAngleEqualsSmart(expectedAxisAngle, actualAxisAngle, 1.0e-10);
      }

      // Test exactly at 0.0
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D firstVector = EuclidCoreRandomTools.generateRandomVector3D(random);
         firstVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));
         Vector3D secondVector = new Vector3D(firstVector);
         secondVector.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));
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
         Vector3D referenceNormal = EuclidCoreRandomTools.generateRandomVector3D(random);
         referenceNormal.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D rotatedNormal = new Vector3D();
         rotatedNormal.setAndNegate(referenceNormal);
         rotatedNormal.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));
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

      // Test getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d normalVector3d)
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D referenceNormal = new Vector3D(0.0, 0.0, 1.0);
         double expectedAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI);
         Vector3D expectedAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, referenceNormal, true);
         AxisAngle expectedAxisAngle = new AxisAngle(expectedAxis, expectedAngle);

         Vector3D rotatedNormal = new Vector3D();
         expectedAxisAngle.transform(referenceNormal, rotatedNormal);
         rotatedNormal.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

         AxisAngle actualAxisAngle = new AxisAngle();
         EuclidGeometryTools.axisAngleFromFirstToSecondVector3D(referenceNormal, rotatedNormal, actualAxisAngle);

         Vector3D actualAxis = new Vector3D(actualAxisAngle.getX(), actualAxisAngle.getY(), actualAxisAngle.getZ());

         assertEquals(1.0, actualAxis.length(), EuclidGeometryTools.ONE_TRILLIONTH);
         assertEquals(0.0, actualAxis.dot(referenceNormal), EuclidGeometryTools.ONE_TRILLIONTH);
         assertEquals(0.0, actualAxis.dot(rotatedNormal), EuclidGeometryTools.ONE_TRILLIONTH);

         assertEquals(0.0, expectedAxis.dot(referenceNormal), EuclidGeometryTools.ONE_TRILLIONTH);
         assertEquals(0.0, expectedAxis.dot(rotatedNormal), EuclidGeometryTools.ONE_TRILLIONTH);

         if (actualAxisAngle.getAngle() * expectedAxisAngle.getAngle() < 0.0)
         {
            expectedAxis.negate();
            expectedAngle = -expectedAngle;
            expectedAxisAngle.set(expectedAxis, expectedAngle);
         }

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
         Point3D lineStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random,
                                                                                               EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         // Rotate line2 around the vector we shifted it along, so it preserves the minimum distance.
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.05, Math.PI - 0.05);
         AxisAngle axisAngleAroundShiftVector = new AxisAngle(orthogonalToLine1, rotationAngle);
         axisAngleAroundShiftVector.transform(lineDirection2);

         // At this point, lineStart1 and lineStart2 are expected to be the closest points.
         expectedPointOnLine1ToPack.set(lineStart1);
         expectedPointOnLine2ToPack.set(lineStart2);

         double actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                             actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, 1.0e-10);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, 1.0e-10);

         // Let's shift lineStart1 and lineStart2 along their respective line direction so they're not the closest points.
         lineStart1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                      actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine1ToPack, actualPointOnLine1ToPack, 1.0e-10);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLine2ToPack, actualPointOnLine2ToPack, 1.0e-10);
      }

      // Test the parallel case. There's an infinite number of solutions but only one minimum distance between the two lines.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         double actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                             actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line direction (the minimum distance should remain the same).
         lineStart1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                      actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Intersection case: the lines are not parallel, but intersecting.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.generateRandomVector3D(random);
         lineDirection1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.5, 10.0));

         // Set the intersection point randomly on line1
         Point3D intersection = new Point3D();
         intersection.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection1, lineStart1);

         // Set both closest points to the intersection
         expectedPointOnLine1ToPack.set(intersection);
         expectedPointOnLine2ToPack.set(intersection);

         // Create line2 such that it intersects line1 at intersection
         Point3D lineStart2 = new Point3D(intersection);
         Vector3D lineDirection2 = EuclidCoreRandomTools.generateRandomVector3D(random);
         lineDirection2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         lineStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, lineStart2);

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
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest within bounds of line segment 1
         expectedPointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineSegmentDirection1, true);
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), orthogonalToLineSegment1,
                                              expectedPointOnLineSegment1);

         // Set the line direction 2 to be the rotation of 1 around the shift direction used to create the expectedPointOnLineSegment2
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.05, Math.PI - 0.05);
         AxisAngle rotationAroundShiftVector = new AxisAngle(orthogonalToLineSegment1, rotationAngle);
         rotationAroundShiftVector.transform(lineSegmentDirection1, lineSegmentDirection2);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);

         double eps = 1.0e-10;
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, eps);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, eps);
      }

      // Parallel case, expecting expectedPointOnLineSegment1 = lineSegmentStart1
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // expectedPointOnLineSegment1 = lineSegmentStart1
         expectedPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineSegmentDirection1, true);
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), orthogonalToLineSegment1,
                                              expectedPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);

         double eps = 1.0e-10;
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, eps);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, eps);

         // Set the end points of the line segment 2 before the expected closest point, so we have expectedClosestPointOnLineSegment2 = lineSegmentEnd2
         double shiftStartFromExpected = EuclidCoreRandomTools.generateRandomDouble(random, -20.0, -10.0);
         double shiftEndFromExpected = EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0);
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
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         expectedPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), shiftVector, expectedPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

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

      // Edge case: both closest points are outside bounds of each line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         expectedPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), shiftVector, expectedPointOnLineSegment1);

         // set the start of the second line segment to the expected closest point
         Point3D lineSegmentStart2 = new Point3D(expectedPointOnLineSegment2);

         // Set the line direction 2 to point somewhat in the same direction as the shift vector
         Vector3D orthogonalToShiftVector = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

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
      // Test for right rectangle, should be half the area of the corresponding rectangle
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = EuclidCoreRandomTools.generateRandomPoint2D(random);
         a.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D b = new Point2D();
         Point2D c = new Point2D();
         Point2D d = new Point2D();

         Vector2D rectangleLength = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 1.0);
         Vector2D rectangleWidth = new Vector2D(-rectangleLength.getY(), rectangleLength.getX());
         double length = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         double width = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
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
         Point2D firstPoint = EuclidCoreRandomTools.generateRandomPoint2D(random);
         Point2D secondPoint = EuclidCoreRandomTools.generateRandomPoint2D(random);

         firstPoint.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         secondPoint.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         double expectedDistance = firstPoint.distance(secondPoint);
         double actualDistance = EuclidGeometryTools.distanceBetweenPoint2Ds(firstPoint.getX(), firstPoint.getY(), secondPoint.getX(), secondPoint.getY());
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
         Point3D lineStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random,
                                                                                               EuclidCoreRandomTools.generateRandomDouble(random, 0.5, 10.0));

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         // Rotate line2 around the vector we shifted it along, so it preserves the minimum distance.
         AxisAngle axisAngleAroundShiftVector = new AxisAngle(orthogonalToLine1, EuclidCoreRandomTools.generateRandomDouble(random, Math.PI));
         RotationMatrix rotationMatrixAroundShiftVector = new RotationMatrix();
         rotationMatrixAroundShiftVector.set(axisAngleAroundShiftVector);
         rotationMatrixAroundShiftVector.transform(lineDirection2);

         // At this point, lineStart1 and lineStart2 are expected to be the closest points.
         closestPointOnLine1.set(lineStart1);
         closestPointOnLine2.set(lineStart2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line direction so they're not the closest points.
         lineStart1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Test the parallel case. There's an infinite number of solutions but only one minimum distance between the two lines.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line direction (the minimum distance should remain the same).
         lineStart1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, lineStart2);

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
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest within bounds of line segment 1
         closestPointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the line direction 2 to be the rotation of 1 around the shift direction used to create the expectedPointOnLineSegment2
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 2.0 * Math.PI);
         AxisAngle rotationAroundShiftVector = new AxisAngle(orthogonalToLineSegment1, rotationAngle);
         rotationAroundShiftVector.transform(lineSegmentDirection1, lineSegmentDirection2);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Parallel case, expecting expectedPointOnLineSegment1 = lineSegmentStart1
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // expectedPointOnLineSegment1 = lineSegmentStart1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Set the end points of the line segment 2 before the expected closest point, so we have expectedClosestPointOnLineSegment2 = lineSegmentEnd2
         double shiftStartFromExpected = EuclidCoreRandomTools.generateRandomDouble(random, -20.0, -10.0);
         double shiftEndFromExpected = EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0);
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
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);
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

      // Edge case: both closest points are outside bounds of each line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         lineSegmentDirection1.sub(lineSegmentEnd1, lineSegmentStart1);
         lineSegmentDirection1.normalize();

         // Put the first closest to the start of line segment 1
         closestPointOnLineSegment1.set(lineSegmentStart1);

         // Create the closest point of line segment 2 such that it reaches out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         double alpha = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, alpha);
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // set the start of the second line segment to the expected closest point
         Point3D lineSegmentStart2 = new Point3D(closestPointOnLineSegment2);

         // Set the line direction 2 to point somewhat in the same direction as the shift vector
         Vector3D orthogonalToShiftVector = EuclidCoreRandomTools.generateRandomOrthogonalVector3d(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected closest point.
         Point3D lineSegmentEnd2 = new Point3D();
         double alpha2 = EuclidCoreRandomTools.generateRandomDouble(random, 0.1, 10.0);
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
}
