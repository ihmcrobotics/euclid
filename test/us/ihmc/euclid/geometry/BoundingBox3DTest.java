package us.ihmc.euclid.geometry;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class BoundingBox3DTest
{
   private static final double EPSILON = EuclidGeometryTools.ONE_TRILLIONTH;
   private static final int ITERATIONS = 1000;

   @Test
   public void testConstructors() throws Exception
   {
      Random random = new Random(345345L);

      Point3D min = new Point3D();
      Point3D max = new Point3D();
      BoundingBox3D boundingBox = new BoundingBox3D();
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Create the min and max coordinates such that they represent a proper bounding box
      min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      boundingBox = new BoundingBox3D(min, max);
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Create the min and max coordinates such that they represent a proper bounding box
      min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      boundingBox = new BoundingBox3D(new BoundingBox3D(min, max));
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Create the min and max coordinates such that they represent a proper bounding box
      min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      double[] minArray = new double[3];
      double[] maxArray = new double[3];
      min.get(minArray);
      max.get(maxArray);
      boundingBox = new BoundingBox3D(minArray, maxArray);
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Create the min and max coordinates such that they represent a proper bounding box
      min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      boundingBox = new BoundingBox3D(min.getX(), min.getY(), min.getZ(), max.getX(), max.getY(), max.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Asserts exception is thrown when attempting to create a bad bounding box
      try
      {
         new BoundingBox3D(new double[] {1.0, 0.0, 0.0}, new double[] {0.0, 0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         new BoundingBox3D(new double[] {0.0, 1.0, 0.0}, new double[] {0.0, 0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         new BoundingBox3D(new double[] {0.0, 0.0, 1.0}, new double[] {0.0, 0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         new BoundingBox3D(new Point3D(1.0, 0.0, 0.0), new Point3D(0.0, 0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         new BoundingBox3D(new Point3D(0.0, 1.0, 0.0), new Point3D(0.0, 0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         new BoundingBox3D(new Point3D(0.0, 0.0, 1.0), new Point3D(0.0, 0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         new BoundingBox3D(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         new BoundingBox3D(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         new BoundingBox3D(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      new BoundingBox3D(new double[] {1.0, 1.0, 1.0}, new double[] {1.0, 1.0, 1.0});
      new BoundingBox3D(new Point3D(1.0, 1.0, 1.0), new Point3D(1.0, 1.0, 1.0));
      new BoundingBox3D(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
   }

   @Test
   public void testSetMin() throws Exception
   {
      Random random = new Random(3242L);
      BoundingBox3D boundingBox = new BoundingBox3D();
      Point3D min = new Point3D();

      min = EuclidCoreRandomTools.generateRandomPoint3D(random, -10.0, 0.0);
      boundingBox.setMin(min);
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);

      min = EuclidCoreRandomTools.generateRandomPoint3D(random, -10.0, 0.0);
      boundingBox.setMin(new double[] {min.getX(), min.getY(), min.getZ()});
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);

      min = EuclidCoreRandomTools.generateRandomPoint3D(random, -10.0, 0.0);
      boundingBox.setMin(min.getX(), min.getY(), min.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);

      // Check exceptions
      try
      {
         boundingBox.setMin(new double[] {1.0, 0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMin(new double[] {0.0, 1.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMin(new double[] {0.0, 0.0, 1.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.setMin(new Point3D(1.0, 0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMin(new Point3D(0.0, 1.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMin(new Point3D(0.0, 0.0, 1.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.setMin(1.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMin(0.0, 1.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMin(0.0, 0.0, 1.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      boundingBox.setMin(new double[] {0.0, 0.0, 0.0});
      boundingBox.setMin(new Point3D(0.0, 0.0, 0.0));
      boundingBox.setMin(0.0, 0.0, 0.0);
   }

   @Test
   public void testSetMax() throws Exception
   {
      Random random = new Random(3242L);
      BoundingBox3D boundingBox = new BoundingBox3D();
      Point3D max = new Point3D();

      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      boundingBox.setMax(max);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      boundingBox.setMax(new double[] {max.getX(), max.getY(), max.getZ()});
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      boundingBox.setMax(max.getX(), max.getY(), max.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      // Check exceptions
      try
      {
         boundingBox.setMax(new double[] {-1.0, 0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMax(new double[] {0.0, -1.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMax(new double[] {0.0, 0.0, -1.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.setMax(new Point3D(-1.0, 0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMax(new Point3D(0.0, -1.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMax(new Point3D(0.0, 0.0, -1.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.setMax(-1.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMax(0.0, -1.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.setMax(0.0, 0.0, -1.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      boundingBox.setMax(new double[] {0.0, 0.0, 0.0});
      boundingBox.setMax(new Point3D(0.0, 0.0, 0.0));
      boundingBox.setMax(0.0, 0.0, 0.0);
   }

   @Test
   public void testSetters() throws Exception
   {
      Random random = new Random(34534L);
      BoundingBox3D boundingBox = new BoundingBox3D();
      Point3D min = new Point3D();
      Point3D max = new Point3D();

      min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      boundingBox.set(min.getX(), min.getY(), min.getZ(), max.getX(), max.getY(), max.getZ());
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      double[] minArray = new double[3];
      double[] maxArray = new double[3];
      min.get(minArray);
      max.get(maxArray);
      boundingBox.set(minArray, maxArray);
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      boundingBox.set(min, max);
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      Point3D center = new Point3D();
      center.add(min, max);
      center.scale(0.5);
      Vector3D halfSize = new Vector3D();
      halfSize.sub(max, min);
      halfSize.scale(0.5);
      boundingBox.set(center, halfSize);
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      boundingBox.set(new BoundingBox3D(min, max));
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);
      // Asserts exception is thrown when attempting to create a bad bounding box
      try
      {
         boundingBox.set(new double[] {1.0, 0.0, 0.0}, new double[] {0.0, 0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(new double[] {0.0, 1.0, 0.0}, new double[] {0.0, 0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(new double[] {0.0, 0.0, 1.0}, new double[] {0.0, 0.0, 0.0});
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.set(new Point3D(1.0, 0.0, 0.0), new Point3D(0.0, 0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(new Point3D(0.0, 1.0, 0.0), new Point3D(0.0, 0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(new Point3D(0.0, 0.0, 1.0), new Point3D(0.0, 0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      try
      {
         boundingBox.set(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }

      boundingBox.set(new double[] {1.0, 1.0, 1.0}, new double[] {1.0, 1.0, 1.0});
      boundingBox.set(new Point3D(1.0, 1.0, 1.0), new Point3D(1.0, 1.0, 1.0));
      boundingBox.set(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
   }

   @Test
   public void testCombine() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox3D boundingBoxOne = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         BoundingBox3D boundingBoxTwo = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         BoundingBox3D combined = new BoundingBox3D();
         combined.combine(boundingBoxOne, boundingBoxTwo);

         // First assert that the resulting bounding box contains all the point of the two bounding bounding boxes
         combined.isInsideInclusive(boundingBoxOne.getMinPoint());
         combined.isInsideInclusive(boundingBoxOne.getMaxPoint());
         combined.isInsideInclusive(boundingBoxTwo.getMinPoint());
         combined.isInsideInclusive(boundingBoxTwo.getMaxPoint());

         // Check that each coordinate of the resulting bounding box comes from one of the two original bounding boxes
         Point3DReadOnly[] originalMinCoordinates = {boundingBoxOne.getMinPoint(), boundingBoxTwo.getMinPoint()};
         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            boolean isMinCoordinateFromOriginal = false;
            for (Point3DReadOnly originalCoordinate : originalMinCoordinates)
            {
               if (combined.getMinPoint().getElement(axisIndex) == originalCoordinate.getElement(axisIndex))
               {
                  isMinCoordinateFromOriginal = true;
                  break;
               }
            }
            assertTrue("Unexpected min coordinate for the combined bounding box, axis index = " + axisIndex, isMinCoordinateFromOriginal);
         }

         Point3DReadOnly[] originalMaxCoordinates = {boundingBoxOne.getMaxPoint(), boundingBoxTwo.getMaxPoint()};
         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            boolean isMaxCoordinateFromOriginal = false;
            for (Point3DReadOnly originalCoordinate : originalMaxCoordinates)
            {
               if (combined.getMaxPoint().getElement(axisIndex) == originalCoordinate.getElement(axisIndex))
               {
                  isMaxCoordinateFromOriginal = true;
                  break;
               }
            }
            assertTrue("Unexpected max coordinate for the combined bounding box, axis index = " + axisIndex, isMaxCoordinateFromOriginal);
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox3D boundingBoxOne = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         BoundingBox3D boundingBoxTwo = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         BoundingBox3D combined = new BoundingBox3D();
         combined.set(boundingBoxOne);
         combined.combine(boundingBoxTwo);

         // First assert that the resulting bounding box contains all the point of the two bounding bounding boxes
         combined.isInsideInclusive(boundingBoxOne.getMinPoint());
         combined.isInsideInclusive(boundingBoxOne.getMaxPoint());
         combined.isInsideInclusive(boundingBoxTwo.getMinPoint());
         combined.isInsideInclusive(boundingBoxTwo.getMaxPoint());

         // Check that each coordinate of the resulting bounding box comes from one of the two original bounding boxes
         Point3DReadOnly[] originalMinCoordinates = {boundingBoxOne.getMinPoint(), boundingBoxTwo.getMinPoint()};
         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            boolean isMinCoordinateFromOriginal = false;
            for (Point3DReadOnly originalCoordinate : originalMinCoordinates)
            {
               if (combined.getMinPoint().getElement(axisIndex) == originalCoordinate.getElement(axisIndex))
               {
                  isMinCoordinateFromOriginal = true;
                  break;
               }
            }
            assertTrue("Unexpected min coordinate for the combined bounding box, axis index = " + axisIndex, isMinCoordinateFromOriginal);
         }

         Point3DReadOnly[] originalMaxCoordinates = {boundingBoxOne.getMaxPoint(), boundingBoxTwo.getMaxPoint()};
         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            boolean isMaxCoordinateFromOriginal = false;
            for (Point3DReadOnly originalCoordinate : originalMaxCoordinates)
            {
               if (combined.getMaxPoint().getElement(axisIndex) == originalCoordinate.getElement(axisIndex))
               {
                  isMaxCoordinateFromOriginal = true;
                  break;
               }
            }
            assertTrue("Unexpected max coordinate for the combined bounding box, axis index = " + axisIndex, isMaxCoordinateFromOriginal);
         }
      }
   }

   @Test
   public void testSetToNaN() throws Exception
   {
      Random random = new Random(453453L);
      BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
      boundingBox3D.setToNaN();
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(boundingBox3D.getMinPoint());
      EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(boundingBox3D.getMaxPoint());
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(453453L);
      BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
      boundingBox3D.setToZero();
      EuclidCoreTestTools.assertTuple3DIsSetToZero(boundingBox3D.getMinPoint());
      EuclidCoreTestTools.assertTuple3DIsSetToZero(boundingBox3D.getMaxPoint());
   }

   @Test
   public void testContainsNaN() throws Exception
   {
      Random random = new Random(23434L);
      for (int i = 0; i < ITERATIONS; i++)
         assertFalse(EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0).containsNaN());

      assertTrue(new BoundingBox3D(Double.NaN, 0, 0, 0, 0, 0).containsNaN());
      assertTrue(new BoundingBox3D(0, Double.NaN, 0, 0, 0, 0).containsNaN());
      assertTrue(new BoundingBox3D(0, 0, Double.NaN, 0, 0, 0).containsNaN());
      assertTrue(new BoundingBox3D(0, 0, 0, Double.NaN, 0, 0).containsNaN());
      assertTrue(new BoundingBox3D(0, 0, 0, 0, Double.NaN, 0).containsNaN());
      assertTrue(new BoundingBox3D(0, 0, 0, 0, 0, Double.NaN).containsNaN());
   }

   @Test
   public void testGetCenterPoint() throws Exception
   {
      Random random = new Random(24324L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D center = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Vector3D halfSize = EuclidCoreRandomTools.generateRandomVector3D(random, 0.0, 10.0);
         BoundingBox3D boundingBox3D = new BoundingBox3D();
         boundingBox3D.set(center, halfSize);
         Point3D actualCenter = new Point3D();
         boundingBox3D.getCenterPoint(actualCenter);
         EuclidCoreTestTools.assertTuple3DEquals(center, actualCenter, EPSILON);
         Vector3D centerToMin = new Vector3D();
         centerToMin.sub(boundingBox3D.getMinPoint(), center);
         Vector3D centerToMax = new Vector3D();
         centerToMax.sub(boundingBox3D.getMaxPoint(), center);

         centerToMax.negate();
         EuclidCoreTestTools.assertTuple3DEquals(centerToMax, centerToMin, EPSILON);
      }
   }

   @Test
   public void testGetPointGivenParameters() throws Exception
   {
      Random random = new Random(324432L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D expectedPoint = new Point3D();
         Point3D actualPoint = new Point3D();
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         double alpha = EuclidCoreRandomTools.generateRandomDouble(random, 10.0);

         expectedPoint.interpolate(boundingBox3D.getMinPoint(), boundingBox3D.getMaxPoint(), alpha);
         boundingBox3D.getPointGivenParameters(alpha, alpha, alpha, actualPoint);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPoint, actualPoint, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D actualPoint = new Point3D();
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         Point3D alpha = new Point3D();

         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
            alpha.setElement(axisIndex, 0.0);
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), actualPoint);
            assertEquals(boundingBox3D.getMinPoint().getElement(axisIndex), actualPoint.getElement(axisIndex), EPSILON);

            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
            alpha.setElement(axisIndex, 1.0);
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), actualPoint);
            assertEquals(boundingBox3D.getMaxPoint().getElement(axisIndex), actualPoint.getElement(axisIndex), EPSILON);
         }
      }
   }

   @Test
   public void testGetDiagonalLengthSquared() throws Exception
   {
      Random random = new Random(324234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D center = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Vector3D halfSize = EuclidCoreRandomTools.generateRandomVector3D(random, 0.0, 10.0);
         BoundingBox3D boundingBox3D = new BoundingBox3D();
         boundingBox3D.set(center, halfSize);
         assertEquals(4.0 * halfSize.lengthSquared(), boundingBox3D.getDiagonalLengthSquared(), EPSILON);
      }
   }

   @Test
   public void testIsInsideExclusive() throws Exception
   {
      Random random = new Random(3243L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         Point3DReadOnly minPoint = boundingBox3D.getMinPoint();
         Point3DReadOnly maxPoint = boundingBox3D.getMaxPoint();
         assertFalse(boundingBox3D.isInsideExclusive(minPoint));
         assertFalse(boundingBox3D.isInsideExclusive(maxPoint));
         assertFalse(boundingBox3D.isInsideExclusive(minPoint.getX(), minPoint.getY(), minPoint.getZ()));
         assertFalse(boundingBox3D.isInsideExclusive(maxPoint.getX(), maxPoint.getY(), maxPoint.getZ()));

         Point3D alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, Double.MIN_VALUE, 1.0 - 1.0e-15);
         Point3D query = new Point3D();
         boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
         assertTrue(boundingBox3D.isInsideExclusive(query));
         assertTrue(boundingBox3D.isInsideExclusive(query.getX(), query.getY(), query.getZ()));

         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, Double.MIN_VALUE, 1.0 - 1.0e-15);
            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertFalse(boundingBox3D.isInsideExclusive(query));
            assertFalse(boundingBox3D.isInsideExclusive(query.getX(), query.getY(), query.getZ()));

            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertFalse(boundingBox3D.isInsideExclusive(query));
            assertFalse(boundingBox3D.isInsideExclusive(query.getX(), query.getY(), query.getZ()));

            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, Double.MIN_VALUE, 1.0 - 1.0e-15);
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            query.setElement(axisIndex, minPoint.getElement(axisIndex));
            assertFalse(boundingBox3D.isInsideExclusive(query));
            assertFalse(boundingBox3D.isInsideExclusive(query.getX(), query.getY(), query.getZ()));

            query.setElement(axisIndex, maxPoint.getElement(axisIndex));
            assertFalse(boundingBox3D.isInsideExclusive(query));
            assertFalse(boundingBox3D.isInsideExclusive(query.getX(), query.getY(), query.getZ()));
         }
      }
   }

   @Test
   public void testIsInsideInclusive() throws Exception
   {
      Random random = new Random(3243L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         Point3DReadOnly minPoint = boundingBox3D.getMinPoint();
         Point3DReadOnly maxPoint = boundingBox3D.getMaxPoint();
         assertTrue(boundingBox3D.isInsideInclusive(minPoint));
         assertTrue(boundingBox3D.isInsideInclusive(maxPoint));
         assertTrue(boundingBox3D.isInsideInclusive(minPoint.getX(), minPoint.getY(), minPoint.getZ()));
         assertTrue(boundingBox3D.isInsideInclusive(maxPoint.getX(), maxPoint.getY(), maxPoint.getZ()));

         Point3D alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
         Point3D query = new Point3D();
         boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
         assertTrue(boundingBox3D.isInsideInclusive(query));
         assertTrue(boundingBox3D.isInsideInclusive(query.getX(), query.getY(), query.getZ()));

         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertFalse(boundingBox3D.isInsideInclusive(query));
            assertFalse(boundingBox3D.isInsideInclusive(query.getX(), query.getY(), query.getZ()));

            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertFalse(boundingBox3D.isInsideInclusive(query));
            assertFalse(boundingBox3D.isInsideInclusive(query.getX(), query.getY(), query.getZ()));

            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            query.setElement(axisIndex, minPoint.getElement(axisIndex));
            assertTrue(boundingBox3D.isInsideInclusive(query));
            assertTrue(boundingBox3D.isInsideInclusive(query.getX(), query.getY(), query.getZ()));

            query.setElement(axisIndex, maxPoint.getElement(axisIndex));
            assertTrue(boundingBox3D.isInsideInclusive(query));
            assertTrue(boundingBox3D.isInsideInclusive(query.getX(), query.getY(), query.getZ()));
         }
      }
   }

   @Test
   public void testIsInsideEpsilon() throws Exception
   {
      Random random = new Random(3243L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D center = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Vector3D halfSize = EuclidCoreRandomTools.generateRandomVector3D(random, 0.0, 10.0);
         BoundingBox3D boundingBoxEpsilon = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, halfSize);
         double epsilon = EuclidCoreRandomTools.generateRandomDouble(random, -Math.min(halfSize.getX(), Math.min(halfSize.getY(), halfSize.getZ())), 1.0);
         halfSize.add(epsilon, epsilon, epsilon);
         BoundingBox3D boundingBoxExclusive = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, halfSize);

         Point3DReadOnly minPoint = boundingBoxEpsilon.getMinPoint();
         Point3DReadOnly maxPoint = boundingBoxEpsilon.getMaxPoint();
         assertEquals(boundingBoxExclusive.isInsideExclusive(minPoint), boundingBoxEpsilon.isInsideEpsilon(minPoint, epsilon));
         assertEquals(boundingBoxExclusive.isInsideExclusive(maxPoint), boundingBoxEpsilon.isInsideEpsilon(maxPoint, epsilon));
         assertEquals(boundingBoxExclusive.isInsideExclusive(minPoint),
                      boundingBoxEpsilon.isInsideEpsilon(minPoint.getX(), minPoint.getY(), minPoint.getZ(), epsilon));
         assertEquals(boundingBoxExclusive.isInsideExclusive(maxPoint),
                      boundingBoxEpsilon.isInsideEpsilon(maxPoint.getX(), maxPoint.getY(), maxPoint.getZ(), epsilon));

         Point3D alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
         Point3D query = new Point3D();
         boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
         assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
         assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), query.getZ(), epsilon));

         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
            boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), query.getZ(), epsilon));

            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
            boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), query.getZ(), epsilon));

            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
            boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            query.setElement(axisIndex, minPoint.getElement(axisIndex));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), query.getZ(), epsilon));

            query.setElement(axisIndex, maxPoint.getElement(axisIndex));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query, epsilon));
            assertEquals(boundingBoxExclusive.isInsideExclusive(query), boundingBoxEpsilon.isInsideEpsilon(query.getX(), query.getY(), query.getZ(), epsilon));
         }
      }
   }

   @Test
   public void testIntersectsExclusive() throws Exception
   {
      Random random = new Random(34545L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D center = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Vector3D halfSize = EuclidCoreRandomTools.generateRandomVector3D(random, 0.0, 10.0);
         BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, halfSize);

         Point3D queryCenter = new Point3D();
         Vector3D queryHalfSize = EuclidCoreRandomTools.generateRandomVector3D(random, 0.0, 10.0);
         BoundingBox3D query = new BoundingBox3D();

         double xParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         double yParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         double zParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         boundingBox3D.getPointGivenParameters(xParameter, yParameter, zParameter, queryCenter);
         query.set(queryCenter, queryHalfSize);
         assertTrue(boundingBox3D.intersectsExclusive(query));

         for (double xSign = -1.0; xSign <= 1.0; xSign += 2.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 2.0)
            {
               for (double zSign = -1.0; zSign <= 1.0; zSign += 2.0)
               {
                  halfSize.absolute();
                  halfSize.scale(xSign, ySign, zSign);
                  
                  Vector3D shift = new Vector3D(queryHalfSize);
                  shift.scale(xSign, ySign, zSign);
                  
                  queryCenter.add(center, halfSize);
                  // TODO: Stopped implementing tests right here.
                  
               }
            }
         }
      }
   }
}
