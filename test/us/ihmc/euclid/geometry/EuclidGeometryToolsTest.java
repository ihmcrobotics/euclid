package us.ihmc.euclid.geometry;

import static org.junit.Assert.*;

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
         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineDirection1, true);
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
         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineDirection, true);

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

         Vector3D rotationAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal1, true);
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

         Vector3D rotationAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, firstVector, true);
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

         Vector3D rotationAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, firstVector, true);
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
         Vector3D expectedAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, firstVector, true);
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
         Vector3D expectedAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, firstVector, true);
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
         Vector3D expectedAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, firstVector, true);
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

      // Test getRotationBasedOnNormal(AxisAngle4d rotationToPack, Vector3d
      // normalVector3d)
      for (int i = 0; i < ITERATIONS; i++)
      {
         Vector3D referenceNormal = new Vector3D(0.0, 0.0, 1.0);
         double expectedAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, Math.PI);
         Vector3D expectedAxis = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, referenceNormal, true);
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
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         // Rotate line2 around the vector we shifted it along, so it
         // preserves the minimum distance.
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.05, Math.PI - 0.05);
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
         lineStart1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, lineStart2);

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
         Point3D lineStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         double actualMinimumDistance = EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2,
                                                                                             actualPointOnLine1ToPack, actualPointOnLine2ToPack);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line
         // direction (the minimum distance should remain the same).
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
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection1, true);
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), orthogonalToLineSegment1,
                                              expectedPointOnLineSegment1);

         // Set the line direction 2 to be the rotation of 1 around the shift
         // direction used to create the expectedPointOnLineSegment2
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 0.05, Math.PI - 0.05);
         AxisAngle rotationAroundShiftVector = new AxisAngle(orthogonalToLineSegment1, rotationAngle);
         rotationAroundShiftVector.transform(lineSegmentDirection1, lineSegmentDirection2);

         // Set the end points of the line segment 2 around the expected
         // closest point.
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

      // Parallel case, expecting expectedPointOnLineSegment1 =
      // lineSegmentStart1
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
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection1, true);
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), orthogonalToLineSegment1,
                                              expectedPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineSegmentDirection2, expectedPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, expectedPointOnLineSegment2);

         EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                     actualPointOnLineSegment1, actualPointOnLineSegment2);

         double eps = 1.0e-10;
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment1, actualPointOnLineSegment1, eps);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPointOnLineSegment2, actualPointOnLineSegment2, eps);

         // Set the end points of the line segment 2 before the expected
         // closest point, so we have expectedClosestPointOnLineSegment2 =
         // lineSegmentEnd2
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

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), shiftVector, expectedPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected
         // closest point.
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

      // Edge case: both closest points are outside bounds of each line
      // segment
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

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         expectedPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), shiftVector, expectedPointOnLineSegment1);

         // set the start of the second line segment to the expected closest
         // point
         Point3D lineSegmentStart2 = new Point3D(expectedPointOnLineSegment2);

         // Set the line direction 2 to point somewhat in the same direction
         // as the shift vector
         Vector3D orthogonalToShiftVector = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected
         // closest point.
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
      // Test for right rectangle, should be half the area of the
      // corresponding rectangle
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
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         // Rotate line2 around the vector we shifted it along, so it
         // preserves the minimum distance.
         AxisAngle axisAngleAroundShiftVector = new AxisAngle(orthogonalToLine1, EuclidCoreRandomTools.generateRandomDouble(random, Math.PI));
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
         lineStart1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection1, lineStart1);
         lineStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, lineStart2);

         actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Test the parallel case. There's an infinite number of solutions but
      // only one minimum distance between the two lines.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineStart1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineDirection1 = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);

         // Make line2 == line1
         Point3D lineStart2 = new Point3D(lineStart1);
         Vector3D lineDirection2 = new Vector3D(lineDirection1);

         // Shift orthogonally line2 away from line1.
         Vector3D orthogonalToLine1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         lineStart2.scaleAdd(expectedMinimumDistance, orthogonalToLine1, lineStart1);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLine3Ds(lineStart1, lineDirection1, lineStart2, lineDirection2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Let's shift lineStart1 and lineStart2 along their respective line
         // direction (the minimum distance should remain the same).
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
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the line direction 2 to be the rotation of 1 around the shift
         // direction used to create the expectedPointOnLineSegment2
         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, 2.0 * Math.PI);
         AxisAngle rotationAroundShiftVector = new AxisAngle(orthogonalToLineSegment1, rotationAngle);
         rotationAroundShiftVector.transform(lineSegmentDirection1, lineSegmentDirection2);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);
      }

      // Parallel case, expecting expectedPointOnLineSegment1 =
      // lineSegmentStart1
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
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection1, true);
         double expectedMinimumDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         closestPointOnLineSegment2.scaleAdd(expectedMinimumDistance, orthogonalToLineSegment1, closestPointOnLineSegment1);

         // Set the lineSegmentDirection2 = lineSegmentDirection1
         lineSegmentDirection2.set(lineSegmentDirection1);

         // Set the end points of the line segment 2 around the expected
         // closest point.
         Point3D lineSegmentStart2 = new Point3D();
         Point3D lineSegmentEnd2 = new Point3D();
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineSegmentDirection2, closestPointOnLineSegment2);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineSegmentDirection2, closestPointOnLineSegment2);

         double actualMinimumDistance = EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2,
                                                                                             lineSegmentEnd2);
         assertEquals(expectedMinimumDistance, actualMinimumDistance, EPSILON);

         // Set the end points of the line segment 2 before the expected
         // closest point, so we have expectedClosestPointOnLineSegment2 =
         // lineSegmentEnd2
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

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // Set the line direction 2 to orthogonal to the shift vector
         lineSegmentDirection2 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, shiftVector, true);

         // Set the end points of the line segment 2 around the expected
         // closest point.
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

      // Edge case: both closest points are outside bounds of each line
      // segment
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

         // Create the closest point of line segment 2 such that it reaches
         // out of line segment 1
         Vector3D oppositeOflineSegmentDirection1 = new Vector3D();
         oppositeOflineSegmentDirection1.setAndNegate(lineSegmentDirection1);
         Vector3D orthogonalToLineSegment1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection1, true);
         Vector3D shiftVector = new Vector3D();
         double alpha = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         shiftVector.interpolate(orthogonalToLineSegment1, oppositeOflineSegmentDirection1, alpha);
         closestPointOnLineSegment2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), shiftVector, closestPointOnLineSegment1);

         // set the start of the second line segment to the expected closest
         // point
         Point3D lineSegmentStart2 = new Point3D(closestPointOnLineSegment2);

         // Set the line direction 2 to point somewhat in the same direction
         // as the shift vector
         Vector3D orthogonalToShiftVector = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, shiftVector, true);
         lineSegmentDirection2.interpolate(shiftVector, orthogonalToShiftVector, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));

         // Set the end points of the line segment 2 around the expected
         // closest point.
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

   @Test
   public void testDistanceFromPoint2DToLine2D() throws Exception
   {
      Random random = new Random(243234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine = EuclidCoreRandomTools.generateRandomPoint2D(random);
         pointOnLine.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector2D lineDirection = EuclidCoreRandomTools.generateRandomVector2D(random);
         lineDirection.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector2D orthogonal = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         orthogonal.normalize();
         if (random.nextBoolean())
            orthogonal.negate();

         double expectedDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);

         Point2D query = new Point2D();
         query.scaleAdd(expectedDistance, orthogonal, pointOnLine);
         query.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, query);

         double actualDistance = EuclidGeometryTools.distanceFromPoint2DToLine2D(query, pointOnLine, lineDirection);
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
         Point2D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         EuclidGeometryTools.perpendicularVector2D(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D projection = new Point2D();
         Point2D testPoint = new Point2D();
         double expectedDistance, actualDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentEnd);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);
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
         Point3D start = EuclidCoreRandomTools.generateRandomPoint3D(random);
         start.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D end = EuclidCoreRandomTools.generateRandomPoint3D(random);
         end.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineDirection = new Vector3D();
         lineDirection.sub(end, start);
         // Generate a random vector orthogonal to the line
         Vector3D orthogonalVector = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineDirection, true);
         double expectedDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         // Generate a random point located at an expected distance from the line
         Point3D randomPoint = new Point3D();
         // Randomize on the line
         randomPoint.interpolate(start, end, EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         // Move the point away from the line by the expected distance
         randomPoint.scaleAdd(expectedDistance, orthogonalVector, randomPoint);

         double actualDistance = EuclidGeometryTools.distanceFromPoint3DToLine3D(randomPoint, start, end);
         assertEquals(expectedDistance, actualDistance, 1.0e-12);

         actualDistance = EuclidGeometryTools.distanceFromPoint3DToLine3D(randomPoint, start, lineDirection);
         assertEquals(expectedDistance, actualDistance, 1.0e-12);
      }
   }

   @Test
   public void testDistanceFromPoint3DToLineSegment3D() throws Exception
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection, true);
         Point3D projection = new Point3D();
         Point3D testPoint = new Point3D();
         double expectedDistance, actualDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedDistance = projection.distance(testPoint);
         actualDistance = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
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
         Point3D pointOnPlane = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random,
                                                                                            EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal, true);
         Point3D secondPointOnPlane = new Point3D();
         secondPointOnPlane.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), parallelToPlane, pointOnPlane);

         double expectedDistance = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);
         Point3D point = new Point3D();
         point.scaleAdd(expectedDistance / planeNormal.length(), planeNormal, secondPointOnPlane);

         double actualDistance = EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, pointOnPlane, planeNormal);
         assertEquals(expectedDistance, actualDistance, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testDistanceSquaredFromPoint2DToLineSegment2D() throws Exception
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         EuclidGeometryTools.perpendicularVector2D(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D projection = new Point2D();
         Point2D testPoint = new Point2D();
         double expectedSquaredDistance, actualSquaredDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(testPoint.getX(), testPoint.getY(), lineSegmentStart.getX(),
                                                                                               lineSegmentStart.getY(), lineSegmentEnd.getX(),
                                                                                               lineSegmentEnd.getY());
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(testPoint.getX(), testPoint.getY(), lineSegmentStart.getX(),
                                                                                               lineSegmentStart.getY(), lineSegmentEnd.getX(),
                                                                                               lineSegmentEnd.getY());
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentEnd);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(testPoint.getX(), testPoint.getY(), lineSegmentStart.getX(),
                                                                                               lineSegmentStart.getY(), lineSegmentEnd.getX(),
                                                                                               lineSegmentEnd.getY());
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);
      }
   }

   @Test
   public void testDistanceSquaredFromPoint3DToLineSegment3D() throws Exception
   {
      Random random = new Random(32423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineSegmentDirection.normalize();
         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection, true);
         Point3D projection = new Point3D();
         Point3D testPoint = new Point3D();
         double expectedSquaredDistance, actualSquaredDistance;

         // Between end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // Before end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
         projection.set(lineSegmentStart);
         expectedSquaredDistance = projection.distanceSquared(testPoint);
         actualSquaredDistance = EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         assertEquals(expectedSquaredDistance, actualSquaredDistance, EuclidGeometryTools.ONE_TRILLIONTH);

         // After end points
         projection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, projection);
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
         Point3D pointOnPlane = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random,
                                                                                            EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal, true);
         Point3D randomLinePlaneIntersection = new Point3D();
         randomLinePlaneIntersection.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3D lineDirection = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);

         // Create the two endPoints on each side of the plane:
         endPoint0.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         assertTrue(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertTrue(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Create the two endPoints on one side of the plane:
         endPoint0.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Create the two endPoints on the other side of the plane:
         endPoint0.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
         assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));

         // Annoying case 1: endPoint0 == endPoint1 => should return false whether the endPoints are on plane or not.
         endPoint0.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
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

      Point3D randomLinePlaneIntersection = EuclidCoreRandomTools.generateRandomPoint3D(random);
      randomLinePlaneIntersection.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
      randomLinePlaneIntersection.setZ(0.0);

      Vector3D lineDirection = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);
      // Ensure that the line direction and the plane normal are somewhat pointing the same direction.
      if (lineDirection.dot(planeNormal) < 0.0)
         lineDirection.negate();

      endPoint0.set(randomLinePlaneIntersection);
      endPoint1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, randomLinePlaneIntersection);
      assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
      assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));
      endPoint0.set(randomLinePlaneIntersection);
      endPoint1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, randomLinePlaneIntersection);
      assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1));
      assertFalse(EuclidGeometryTools.doesLineSegment3DIntersectPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0));
   }

   @Test
   public void testDoLineSegment2DsIntersect() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D pointOnLineSegment1 = new Point2D();
         pointOnLineSegment1.interpolate(lineSegmentStart1, lineSegmentEnd1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));

         Vector2D lineDirection2 = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection2, pointOnLineSegment1);
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertFalse(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test intersection at one of the end points
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D pointOnLineSegment1 = new Point2D(lineSegmentStart1);

         Vector2D lineDirection2 = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, pointOnLineSegment1);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection2, pointOnLineSegment1);
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2));
         assertTrue(EuclidGeometryTools.doLineSegment2DsIntersect(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2));
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = EuclidCoreRandomTools.generateRandomDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.generateRandomDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if ((0.0 < alpha1 && alpha1 < 1.0) || (0.0 < alpha2 && alpha2 < 1.0) || alpha1 * alpha2 < 0.0)
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

         double distance = EuclidCoreRandomTools.generateRandomDouble(random, 1.0e-10, 10.0);
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
      {
         Point2D start1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         start1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D end1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         end1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D start2 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         start2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D end2 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         end2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector2D vector1 = new Vector2D();
         vector1.sub(end1, start1);
         Vector2D vector2 = new Vector2D();
         vector2.sub(end2, start2);

         double expectedDotProduct = vector1.dot(vector2);
         double actualDotProduct = EuclidGeometryTools.dotProduct(start1, end1, start2, end2);
         assertEquals(expectedDotProduct, actualDotProduct, EPSILON);
      }
   }

   @Test
   public void testIntersectionBetweenLine2DAndLineSegment2D() throws Exception
   {
      double epsilon = EPSILON;
      Random random = new Random(23423L);
      Point2D actualIntersection = new Point2D();
      boolean success;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));

         Point2D pointOnLine = new Point2D(expectedIntersection);
         Vector2D lineDirection = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 1.0);

         // Expecting intersection
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Make the intersection happen outside the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 1.0);

         Point2D lineLineIntersection = new Point2D();
         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 2.0));
         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertFalse(success);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertFalse(success);

         lineLineIntersection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, -1.0, 0.0));
         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, lineLineIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertFalse(success);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertFalse(success);
      }

      // Make the intersection happen on each end point of the line segment
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D pointOnLine = new Point2D();
         Vector2D lineDirection = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 1.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.set(lineSegmentStart);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         expectedIntersection.set(lineSegmentEnd);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Make the line segment and the line parallel not collinear.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         Vector2D orthogonal = new Vector2D(-lineDirection.getY(), lineDirection.getY());

         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), orthogonal, pointOnLine);
         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertFalse(success);
      }

      // Make the line segment and the line collinear.
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D pointOnLine = new Point2D(lineSegmentStart);
         Vector2D lineDirection = new Vector2D();
         lineDirection.sub(lineSegmentEnd, lineSegmentStart);
         lineDirection.normalize();
         if (random.nextBoolean())
            lineDirection.negate();

         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, pointOnLine);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentStart, lineSegmentEnd,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentStart, actualIntersection, epsilon);
         success = EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(pointOnLine, lineDirection, lineSegmentEnd, lineSegmentStart,
                                                                                 actualIntersection);
         assertTrue(success);
         EuclidCoreTestTools.assertTuple2DEquals(lineSegmentEnd, actualIntersection, epsilon);
      }
   }

   @Test
   public void testIntersectionBetweenLine3DAndPlane3D() throws Exception
   {
      Random random = new Random(1176L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D parallelToPlane = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal, true);

         Point3D expectedIntersection = new Point3D();
         expectedIntersection.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3D lineDirection = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D pointOnLine = new Point3D();
         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, expectedIntersection);

         Point3D actualIntersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection);

         double epsilon = EuclidGeometryTools.ONE_TRILLIONTH;
         if (Math.abs(lineDirection.angle(planeNormal)) > Math.PI / 2.0 - 0.001)
            epsilon = 1.0e-11; // Loss of precision when the line direction and the plane normal are almost orthogonal.

         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Try parallel lines to plane
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector3D lineDirection = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal, false);
         Point3D pointOnLine = new Point3D();
         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, pointOnPlane);

         Point3D actualIntersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection);
         assertNull(actualIntersection);

         pointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 1.0), planeNormal, pointOnLine);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane, planeNormal, pointOnLine, lineDirection);
         assertNull(actualIntersection);
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
         Point3D pointOnPlane = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D parallelToPlane = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal, true);

         Point3D expectedIntersection = new Point3D();
         expectedIntersection.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Vector3D lineDirection = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         // Expecting an actual intersection
         endPoint0.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection, expectedIntersection);
         Point3D actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, 1.0e-11);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0);
         EuclidCoreTestTools.assertTuple3DEquals(expectedIntersection, actualIntersection, 1.0e-11);

         // Expecting no intersection
         endPoint0.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         endPoint1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection, expectedIntersection);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint1, endPoint0);
         assertNull(actualIntersection);
      }

      // Try parallel lines to plane
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector3D lineDirection = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal, false);
         endPoint0.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, pointOnPlane);
         endPoint1.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, pointOnPlane);

         Point3D actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);

         double distanceAwayFromPlane = EuclidCoreRandomTools.generateRandomDouble(random, 1.0);
         endPoint0.scaleAdd(distanceAwayFromPlane, planeNormal, endPoint0);
         endPoint1.scaleAdd(distanceAwayFromPlane, planeNormal, endPoint0);
         actualIntersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlane, planeNormal, endPoint0, endPoint1);
         assertNull(actualIntersection);
      }
   }

   @Test
   public void testIntersectionBetweenTwoLine2Ds() throws Exception
   {
      Random random = new Random(1176L);
      double epsilon = EPSILON;

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         pointOnLine1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector2D lineDirection1 = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 10.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection1, pointOnLine1);

         Vector2D lineDirection2 = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 10.0);
         Point2D pointOnLine2 = new Point2D(expectedIntersection);

         if (Math.abs(lineDirection1.dot(lineDirection2) / lineDirection1.length() / lineDirection2.length()) > 1.0 - 0.0005)
            epsilon = 1.0e-11; // Loss of precision for small angles between the two lines.
         else
            epsilon = 1.0e-12;
         Point2D actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, pointOnLine2);
         actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }

      // Test when parallel but not collinear
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         pointOnLine1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector2D lineDirection1 = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 10.0);

         Vector2D lineDirection2 = new Vector2D(lineDirection1);
         if (random.nextBoolean())
            lineDirection2.negate();
         Point2D pointOnLine2 = new Point2D(pointOnLine1);

         Vector2D orthogonal = new Vector2D(-lineDirection1.getY(), lineDirection1.getX());

         pointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, pointOnLine2);
         pointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, pointOnLine2);
         Point2D actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         assertNull(actualIntersection);
      }

      // Test when collinear
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         pointOnLine1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector2D lineDirection1 = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 10.0);

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.set(pointOnLine1);

         Vector2D lineDirection2 = new Vector2D(lineDirection1);
         Point2D pointOnLine2 = new Point2D(expectedIntersection);

         Point2D actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

         pointOnLine2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection2, pointOnLine2);
         actualIntersection = EuclidGeometryTools.intersectionBetweenTwoLine2Ds(pointOnLine1, lineDirection1, pointOnLine2, lineDirection2);
         EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      }
   }

   @Test
   public void testIntersectionBetweenTwoLineSegment2Ds() throws Exception
   {
      Random random = new Random(3242L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D expectedIntersection = new Point2D();
         expectedIntersection.interpolate(lineSegmentStart1, lineSegmentEnd1, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));

         Vector2D lineDirection2 = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test intersection at one of the end points
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D expectedIntersection = new Point2D(lineSegmentStart1);

         Vector2D lineDirection2 = EuclidCoreRandomTools.generateRandomVector2DWithFixedLength(random, 1.0);

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         // Not expecting intersection
         lineSegmentStart2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0), lineDirection2, expectedIntersection);
         lineSegmentEnd2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0), lineDirection2, expectedIntersection);
         assertAllCombinationsOfTwoLineSegmentsIntersection(expectedIntersection, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }

      // Test with parallel/collinear line segments
      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D lineSegmentStart2 = new Point2D();
         Point2D lineSegmentEnd2 = new Point2D();

         double alpha1 = EuclidCoreRandomTools.generateRandomDouble(random, 2.0);
         double alpha2 = EuclidCoreRandomTools.generateRandomDouble(random, 2.0);

         // Make the second line segment collinear to the first one
         lineSegmentStart2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha1);
         lineSegmentEnd2.interpolate(lineSegmentStart1, lineSegmentEnd1, alpha2);

         if ((0.0 < alpha1 && alpha1 < 1.0) || (0.0 < alpha2 && alpha2 < 1.0) || alpha1 * alpha2 < 0.0)
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

         double distance = EuclidCoreRandomTools.generateRandomDouble(random, 1.0e-10, 10.0);
         lineSegmentStart2.scaleAdd(distance, orthogonal, lineSegmentStart2);
         lineSegmentEnd2.scaleAdd(distance, orthogonal, lineSegmentEnd2);
         assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(false, lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2);
      }
   }

   private void assertOnlyExistenceOfTwoLineSegmentsIntersectionAllCombinations(boolean intersectionExist, Point2D lineSegmentStart1, Point2D lineSegmentEnd1,
                                                                                Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {
      boolean success;
      Point2D actualIntersection = new Point2D();

      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                         actualIntersection);
      assertTrue(success == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2,
                                                                         actualIntersection);
      assertTrue(success == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2,
                                                                         actualIntersection);
      assertTrue(success == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2,
                                                                         actualIntersection);
      assertTrue(success == intersectionExist);

      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart2, lineSegmentEnd2, lineSegmentStart1, lineSegmentEnd1,
                                                                         actualIntersection);
      assertTrue(success == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart2, lineSegmentEnd2, lineSegmentEnd1, lineSegmentStart1,
                                                                         actualIntersection);
      assertTrue(success == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentEnd2, lineSegmentStart2, lineSegmentStart1, lineSegmentEnd1,
                                                                         actualIntersection);
      assertTrue(success == intersectionExist);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentEnd2, lineSegmentStart2, lineSegmentEnd1, lineSegmentStart1,
                                                                         actualIntersection);
      assertTrue(success == intersectionExist);
   }

   private void assertAllCombinationsOfTwoLineSegmentsIntersection(Point2D expectedIntersection, Point2D lineSegmentStart1, Point2D lineSegmentEnd1,
                                                                   Point2D lineSegmentStart2, Point2D lineSegmentEnd2)
   {
      double epsilon = EuclidGeometryTools.ONE_TRILLIONTH;

      Vector2D direction1 = new Vector2D();
      direction1.sub(lineSegmentEnd1, lineSegmentStart1);
      Vector2D direction2 = new Vector2D();
      direction2.sub(lineSegmentEnd2, lineSegmentStart2);

      if (Math.abs(direction1.dot(direction2)) > 1.0 - 0.0001)
         epsilon = 1.0e-10;

      boolean success;
      Point2D actualIntersection = new Point2D();

      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentStart2, lineSegmentEnd2,
                                                                         actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart1, lineSegmentEnd1, lineSegmentEnd2, lineSegmentStart2,
                                                                         actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentStart2, lineSegmentEnd2,
                                                                         actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentEnd1, lineSegmentStart1, lineSegmentEnd2, lineSegmentStart2,
                                                                         actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);

      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart2, lineSegmentEnd2, lineSegmentStart1, lineSegmentEnd1,
                                                                         actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentStart2, lineSegmentEnd2, lineSegmentEnd1, lineSegmentStart1,
                                                                         actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentEnd2, lineSegmentStart2, lineSegmentStart1, lineSegmentEnd1,
                                                                         actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
      success = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(lineSegmentEnd2, lineSegmentStart2, lineSegmentEnd1, lineSegmentStart1,
                                                                         actualIntersection);
      assertTrue(success);
      EuclidCoreTestTools.assertTuple2DEquals(expectedIntersection, actualIntersection, epsilon);
   }

   @Test
   public void testIntersectionBetweenTwoPlane3Ds() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnPlane1 = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnPlane1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D planeNormal1 = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random,
                                                                                             EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

         Vector3D firstParallelToPlane1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal1, true);
         Vector3D secondParallelToPlane1 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal1, true);

         Point3D firstPointOnIntersection = new Point3D();
         Point3D secondPointOnIntersection = new Point3D();
         firstPointOnIntersection.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), firstParallelToPlane1, pointOnPlane1);
         secondPointOnIntersection.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), secondParallelToPlane1, firstPointOnIntersection);

         Vector3D expectedIntersectionDirection = new Vector3D();
         expectedIntersectionDirection.sub(secondPointOnIntersection, firstPointOnIntersection);
         expectedIntersectionDirection.normalize();

         double rotationAngle = EuclidCoreRandomTools.generateRandomDouble(random, Math.PI);
         AxisAngle rotationAxisAngle = new AxisAngle(expectedIntersectionDirection, rotationAngle);
         RotationMatrix rotationMatrix = new RotationMatrix();
         rotationMatrix.set(rotationAxisAngle);

         Vector3D planeNormal2 = new Vector3D();
         rotationMatrix.transform(planeNormal1, planeNormal2);
         planeNormal2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));
         Point3D pointOnPlane2 = new Point3D();

         Vector3D parallelToPlane2 = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal2, true);
         pointOnPlane2.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), parallelToPlane2, firstPointOnIntersection);

         Point3D actualPointOnIntersection = new Point3D();
         Vector3D actualIntersectionDirection = new Vector3D();

         boolean success = EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(pointOnPlane1, planeNormal1, pointOnPlane2, planeNormal2,
                                                                              actualPointOnIntersection, actualIntersectionDirection);
         boolean areParallel = EuclidGeometryTools.areVector3DsCollinear(planeNormal1, planeNormal2, 1.0e-6);
         assertNotEquals(areParallel, success);
         if (areParallel)
            continue;

         if (expectedIntersectionDirection.dot(actualIntersectionDirection) < 0.0)
            expectedIntersectionDirection.negate();

         String message = "Angle between vectors " + expectedIntersectionDirection.angle(actualIntersectionDirection);
         assertTrue(message, EuclidGeometryTools.areVector3DsCollinear(expectedIntersectionDirection, actualIntersectionDirection, 1.0e-7));
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
         Point2D point0 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         point0.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D point1 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         point1.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D point2 = EuclidCoreRandomTools.generateRandomPoint2D(random);
         point2.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

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
   }

   @Test
   public void testIsPoint2DInsideTriangleABC() throws Exception
   {
      Point2D inside = new Point2D();
      Point2D outside = new Point2D();
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D a = EuclidCoreRandomTools.generateRandomPoint2D(random);
         a.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D b = EuclidCoreRandomTools.generateRandomPoint2D(random);
         b.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D c = EuclidCoreRandomTools.generateRandomPoint2D(random);
         c.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(a, a, b, c));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(a, c, b, a));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(b, a, b, c));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(b, c, b, a));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(c, a, b, c));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(c, c, b, a));

         inside.interpolate(a, b, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         inside.interpolate(inside, c, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, a, b, c));
         assertTrue(EuclidGeometryTools.isPoint2DInsideTriangleABC(inside, c, b, a));

         outside.interpolate(a, b, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
         outside.interpolate(outside, c, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, a, b, c));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
         outside.interpolate(outside, c, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, a, b, c));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         outside.interpolate(outside, c, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, a, b, c));
         assertFalse(EuclidGeometryTools.isPoint2DInsideTriangleABC(outside, c, b, a));

         outside.interpolate(a, b, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         outside.interpolate(outside, c, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
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
   public void testIsPoint2DOnSideOfLine2D() throws Exception
   {
      Random random = new Random(2342L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D pointOnLine = EuclidCoreRandomTools.generateRandomPoint2D(random);
         pointOnLine.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector2D lineDirection = EuclidCoreRandomTools.generateRandomVector2D(random);
         lineDirection.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Vector2D orthogonalToTheLeft = EuclidGeometryTools.perpendicularVector2D(lineDirection);
         orthogonalToTheLeft.normalize();
         Vector2D orthogonalToTheRight = new Vector2D();
         orthogonalToTheRight.setAndNegate(orthogonalToTheLeft);

         Point2D secondPointOnLine = new Point2D();
         secondPointOnLine.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, pointOnLine);

         Point2D query = new Point2D();
         double alpha = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0);

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
         Vector3D expectedPlaneNormal = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, 1.0);

         Point3D firstPointOnPlane = EuclidCoreRandomTools.generateRandomPoint3D(random);
         firstPointOnPlane.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D secondPointOnPlane = new Point3D();
         Point3D thirdPointOnPlane = new Point3D();

         Vector3D secondOrthogonalToNormal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, expectedPlaneNormal, true);
         Vector3D thirdOrthogonalToNormal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, expectedPlaneNormal, true);

         secondPointOnPlane.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0), secondOrthogonalToNormal, firstPointOnPlane);
         thirdPointOnPlane.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0), thirdOrthogonalToNormal, firstPointOnPlane);

         Vector3D actualPlaneNormal = EuclidGeometryTools.normal3DFromThreePoint3Ds(firstPointOnPlane, secondPointOnPlane, thirdPointOnPlane);

         if (expectedPlaneNormal.dot(actualPlaneNormal) < 0.0)
            actualPlaneNormal.negate();

         EuclidCoreTestTools.assertTuple3DEquals(expectedPlaneNormal, actualPlaneNormal, EPSILON);
      }
   }

   @Test
   public void testOrthogonalProjectionOnLine2D() throws Exception
   {
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D firstPointOnLine = EuclidCoreRandomTools.generateRandomPoint2D(random);
         firstPointOnLine.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D secondPointOnLine = EuclidCoreRandomTools.generateRandomPoint2D(random);
         secondPointOnLine.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));

         Point2D expectionProjection = new Point2D();
         expectionProjection.interpolate(firstPointOnLine, secondPointOnLine, EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector2D perpendicularToLineDirection = new Vector2D();
         perpendicularToLineDirection.sub(secondPointOnLine, firstPointOnLine);
         perpendicularToLineDirection.normalize();
         EuclidGeometryTools.perpendicularVector2D(perpendicularToLineDirection, perpendicularToLineDirection);

         Point2D testPoint = new Point2D();
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), perpendicularToLineDirection, expectionProjection);

         Point2D actualProjection = EuclidGeometryTools.orthogonalProjectionOnLine2D(testPoint, firstPointOnLine, secondPointOnLine);
         EuclidCoreTestTools.assertTuple2DEquals(expectionProjection, actualProjection, EPSILON);
      }
   }

   @Test
   public void testOrthogonalProjectionOnLine3D() throws Exception
   {
      Random random = new Random(1176L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D pointOnLine = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnLine.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineDirection = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random,
                                                                                              EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));
         Point3D expectedProjection = new Point3D();
         expectedProjection.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), lineDirection, pointOnLine);
         Vector3D perpendicularToLineDirection = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineDirection, true);

         Point3D testPoint = new Point3D();
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), perpendicularToLineDirection, expectedProjection);

         Point3D actualProjection = EuclidGeometryTools.orthogonalProjectionOnLine3D(testPoint, pointOnLine, lineDirection);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }
   }

   @Test
   public void testOrthogonalProjectionOnLineSegment2D() throws Exception
   {
      Random random = new Random(232L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point2D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point2D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint2D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector2D orthogonal = new Vector2D();
         orthogonal.sub(lineSegmentEnd, lineSegmentStart);
         EuclidGeometryTools.perpendicularVector2D(orthogonal, orthogonal);
         orthogonal.normalize();
         Point2D expectedProjection = new Point2D();
         Point2D testPoint = new Point2D();

         // Between end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, expectedProjection);
         Point2D actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);

         // Before end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentStart);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple2DEquals(expectedProjection, actualProjection, EPSILON);

         // After end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentEnd);
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
         Point3D lineSegmentStart = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentStart.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Point3D lineSegmentEnd = EuclidCoreRandomTools.generateRandomPoint3D(random);
         lineSegmentEnd.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D lineSegmentDirection = new Vector3D();
         lineSegmentDirection.sub(lineSegmentEnd, lineSegmentStart);
         Vector3D orthogonal = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, lineSegmentDirection, true);
         Point3D expectedProjection = new Point3D();
         Point3D testPoint = new Point3D();

         // Between end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, expectedProjection);
         Point3D actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);

         // Before end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentStart);
         actualProjection = EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(testPoint, lineSegmentStart, lineSegmentEnd);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);

         // After end points
         expectedProjection.interpolate(lineSegmentStart, lineSegmentEnd, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
         testPoint.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), orthogonal, expectedProjection);
         expectedProjection.set(lineSegmentEnd);
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
         Point3D pointOnPlane = EuclidCoreRandomTools.generateRandomPoint3D(random);
         pointOnPlane.scale(EuclidCoreRandomTools.generateRandomDouble(random, 10.0));
         Vector3D planeNormal = EuclidCoreRandomTools.generateRandomVector3DWithFixedLength(random, EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 10.0));

         Vector3D parallelToPlane = EuclidCoreRandomTools.generateRandomOrthogonalVector3D(random, planeNormal, true);
         Point3D expectedProjection = new Point3D();
         expectedProjection.scaleAdd(EuclidCoreRandomTools.generateRandomDouble(random, 10.0), parallelToPlane, pointOnPlane);

         Point3D pointToProject = new Point3D();
         double distanceOffPlane = EuclidCoreRandomTools.generateRandomDouble(random, 10.0);
         pointToProject.scaleAdd(distanceOffPlane, planeNormal, expectedProjection);

         Point3D actualProjection = EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, pointOnPlane, planeNormal);
         EuclidCoreTestTools.assertTuple3DEquals(expectedProjection, actualProjection, EPSILON);
      }
   }

   @Test
   public void testPercentageAlongLineSegment2D() throws Exception
   {
      
   }
}
