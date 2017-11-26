package us.ihmc.euclid.geometry.tools;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment1D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.tools.EuclidCoreTestToolsTest;

public class EuclidGeometryTestToolsTest
{
   private static final double EPSILON = 0.0001;

   @Test
   public void testAssertLine2DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLine2DEquals";
      Class<Line2D> argumentsClass = Line2D.class;

      {
         Line2D expected = null;
         Line2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line2D expected = EuclidGeometryRandomTools.generateRandomLine2D(random);
         Line2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line2D expected = null;
         Line2D actual = EuclidGeometryRandomTools.generateRandomLine2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line2D expected = EuclidGeometryRandomTools.generateRandomLine2D(random);
         Line2D actual = EuclidGeometryRandomTools.generateRandomLine2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line2D expected = EuclidGeometryRandomTools.generateRandomLine2D(random);
         Line2D actual = new Line2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertLine2DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLine2DGeometricallyEquals";
      Class<Line2D> argumentsClass = Line2D.class;

      {
         Line2D expected = null;
         Line2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line2D expected = EuclidGeometryRandomTools.generateRandomLine2D(random);
         Line2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line2D expected = null;
         Line2D actual = EuclidGeometryRandomTools.generateRandomLine2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line2D expected = EuclidGeometryRandomTools.generateRandomLine2D(random);
         Line2D actual = EuclidGeometryRandomTools.generateRandomLine2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line2D expected = EuclidGeometryRandomTools.generateRandomLine2D(random);
         Line2D actual = new Line2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertLine3DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLine3DEquals";
      Class<Line3D> argumentsClass = Line3D.class;

      {
         Line3D expected = null;
         Line3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line3D expected = EuclidGeometryRandomTools.generateRandomLine3D(random);
         Line3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line3D expected = null;
         Line3D actual = EuclidGeometryRandomTools.generateRandomLine3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line3D expected = EuclidGeometryRandomTools.generateRandomLine3D(random);
         Line3D actual = EuclidGeometryRandomTools.generateRandomLine3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line3D expected = EuclidGeometryRandomTools.generateRandomLine3D(random);
         Line3D actual = new Line3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertLine3DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLine3DGeometricallyEquals";
      Class<Line3D> argumentsClass = Line3D.class;

      {
         Line3D expected = null;
         Line3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line3D expected = EuclidGeometryRandomTools.generateRandomLine3D(random);
         Line3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line3D expected = null;
         Line3D actual = EuclidGeometryRandomTools.generateRandomLine3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line3D expected = EuclidGeometryRandomTools.generateRandomLine3D(random);
         Line3D actual = EuclidGeometryRandomTools.generateRandomLine3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Line3D expected = EuclidGeometryRandomTools.generateRandomLine3D(random);
         Line3D actual = new Line3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertLineSegment1DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLineSegment1DEquals";
      Class<LineSegment1D> argumentsClass = LineSegment1D.class;

      {
         LineSegment1D expected = null;
         LineSegment1D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment1D expected = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         LineSegment1D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment1D expected = null;
         LineSegment1D actual = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment1D expected = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         LineSegment1D actual = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment1D expected = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         LineSegment1D actual = new LineSegment1D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertLineSegment1DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLineSegment1DGeometricallyEquals";
      Class<LineSegment1D> argumentsClass = LineSegment1D.class;

      {
         LineSegment1D expected = null;
         LineSegment1D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment1D expected = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         LineSegment1D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment1D expected = null;
         LineSegment1D actual = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment1D expected = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         LineSegment1D actual = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment1D expected = EuclidGeometryRandomTools.generateRandomLineSegment1D(random);
         LineSegment1D actual = new LineSegment1D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertLineSegment2DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLineSegment2DEquals";
      Class<LineSegment2D> argumentsClass = LineSegment2D.class;

      {
         LineSegment2D expected = null;
         LineSegment2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment2D expected = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         LineSegment2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment2D expected = null;
         LineSegment2D actual = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment2D expected = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         LineSegment2D actual = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment2D expected = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         LineSegment2D actual = new LineSegment2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertLineSegment2DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLineSegment2DGeometricallyEquals";
      Class<LineSegment2D> argumentsClass = LineSegment2D.class;

      {
         LineSegment2D expected = null;
         LineSegment2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment2D expected = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         LineSegment2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment2D expected = null;
         LineSegment2D actual = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment2D expected = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         LineSegment2D actual = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment2D expected = EuclidGeometryRandomTools.generateRandomLineSegment2D(random);
         LineSegment2D actual = new LineSegment2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertLineSegment3DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLineSegment3DEquals";
      Class<LineSegment3D> argumentsClass = LineSegment3D.class;

      {
         LineSegment3D expected = null;
         LineSegment3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment3D expected = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         LineSegment3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment3D expected = null;
         LineSegment3D actual = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment3D expected = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         LineSegment3D actual = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment3D expected = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         LineSegment3D actual = new LineSegment3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertLineSegment3DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertLineSegment3DGeometricallyEquals";
      Class<LineSegment3D> argumentsClass = LineSegment3D.class;

      {
         LineSegment3D expected = null;
         LineSegment3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment3D expected = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         LineSegment3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment3D expected = null;
         LineSegment3D actual = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment3D expected = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         LineSegment3D actual = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         LineSegment3D expected = EuclidGeometryRandomTools.generateRandomLineSegment3D(random);
         LineSegment3D actual = new LineSegment3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertBoundingBox2DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertBoundingBox2DEquals";
      Class<BoundingBox2D> argumentsClass = BoundingBox2D.class;

      {
         BoundingBox2D expected = null;
         BoundingBox2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox2D expected = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         BoundingBox2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox2D expected = null;
         BoundingBox2D actual = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox2D expected = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         BoundingBox2D actual = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox2D expected = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         BoundingBox2D actual = new BoundingBox2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertBoundingBox2DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertBoundingBox2DGeometricallyEquals";
      Class<BoundingBox2D> argumentsClass = BoundingBox2D.class;

      {
         BoundingBox2D expected = null;
         BoundingBox2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox2D expected = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         BoundingBox2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox2D expected = null;
         BoundingBox2D actual = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox2D expected = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         BoundingBox2D actual = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox2D expected = EuclidGeometryRandomTools.generateRandomBoundingBox2D(random);
         BoundingBox2D actual = new BoundingBox2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertBoundingBox3DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertBoundingBox3DEquals";
      Class<BoundingBox3D> argumentsClass = BoundingBox3D.class;

      {
         BoundingBox3D expected = null;
         BoundingBox3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox3D expected = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         BoundingBox3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox3D expected = null;
         BoundingBox3D actual = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox3D expected = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         BoundingBox3D actual = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox3D expected = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         BoundingBox3D actual = new BoundingBox3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertBoundingBox3DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertBoundingBox3DGeometricallyEquals";
      Class<BoundingBox3D> argumentsClass = BoundingBox3D.class;

      {
         BoundingBox3D expected = null;
         BoundingBox3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox3D expected = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         BoundingBox3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox3D expected = null;
         BoundingBox3D actual = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox3D expected = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         BoundingBox3D actual = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         BoundingBox3D expected = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random);
         BoundingBox3D actual = new BoundingBox3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertOrientation2DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertOrientation2DEquals";
      Class<Orientation2D> argumentsClass = Orientation2D.class;

      {
         Orientation2D expected = null;
         Orientation2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Orientation2D expected = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         Orientation2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Orientation2D expected = null;
         Orientation2D actual = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Orientation2D expected = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         Orientation2D actual = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Orientation2D expected = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         Orientation2D actual = new Orientation2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertOrientation2DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertOrientation2DGeometricallyEquals";
      Class<Orientation2D> argumentsClass = Orientation2D.class;

      {
         Orientation2D expected = null;
         Orientation2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Orientation2D expected = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         Orientation2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Orientation2D expected = null;
         Orientation2D actual = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Orientation2D expected = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         Orientation2D actual = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Orientation2D expected = EuclidGeometryRandomTools.generateRandomOrientation2D(random);
         Orientation2D actual = new Orientation2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertPlane3DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertPlane3DEquals";
      Class<Plane3D> argumentsClass = Plane3D.class;

      {
         Plane3D expected = null;
         Plane3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Plane3D expected = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         Plane3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Plane3D expected = null;
         Plane3D actual = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Plane3D expected = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         Plane3D actual = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Plane3D expected = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         Plane3D actual = new Plane3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertPlane3DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertPlane3DGeometricallyEquals";
      Class<Plane3D> argumentsClass = Plane3D.class;

      {
         Plane3D expected = null;
         Plane3D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Plane3D expected = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         Plane3D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Plane3D expected = null;
         Plane3D actual = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Plane3D expected = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         Plane3D actual = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Plane3D expected = EuclidGeometryRandomTools.generateRandomPlane3D(random);
         Plane3D actual = new Plane3D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertPose2DEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertPose2DEquals";
      Class<Pose2D> argumentsClass = Pose2D.class;

      {
         Pose2D expected = null;
         Pose2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Pose2D expected = EuclidGeometryRandomTools.generateRandomPose2D(random);
         Pose2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Pose2D expected = null;
         Pose2D actual = EuclidGeometryRandomTools.generateRandomPose2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Pose2D expected = EuclidGeometryRandomTools.generateRandomPose2D(random);
         Pose2D actual = EuclidGeometryRandomTools.generateRandomPose2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Pose2D expected = EuclidGeometryRandomTools.generateRandomPose2D(random);
         Pose2D actual = new Pose2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   @Test
   public void testAssertPose2DGeometricallyEquals() throws Throwable
   {
      Random random = new Random(453453);
      String methodName = "assertPose2DGeometricallyEquals";
      Class<Pose2D> argumentsClass = Pose2D.class;

      {
         Pose2D expected = null;
         Pose2D actual = null;
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Pose2D expected = EuclidGeometryRandomTools.generateRandomPose2D(random);
         Pose2D actual = null;
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Pose2D expected = null;
         Pose2D actual = EuclidGeometryRandomTools.generateRandomPose2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Pose2D expected = EuclidGeometryRandomTools.generateRandomPose2D(random);
         Pose2D actual = EuclidGeometryRandomTools.generateRandomPose2D(random);
         assertAssertionMethodsBehaveProperly(true, methodName, argumentsClass, expected, actual, EPSILON);
      }

      {
         Pose2D expected = EuclidGeometryRandomTools.generateRandomPose2D(random);
         Pose2D actual = new Pose2D(expected);
         assertAssertionMethodsBehaveProperly(false, methodName, argumentsClass, expected, actual, EPSILON);
      }
   }

   private static void assertAssertionMethodsBehaveProperly(boolean failExpected, String methodName, Class<?> argumentsClass, Object expected, Object actual,
                                                            double epsilon)
         throws Throwable
   {
      EuclidCoreTestToolsTest.assertAssertionMethodsBehaveProperly(EuclidGeometryTestTools.class, failExpected, methodName, argumentsClass, expected, actual, epsilon);
   }
}
