package us.ihmc.euclid.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
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
   public void testStaticConstructors() throws Exception
   {
      Random random = new Random(32443L);

      Point3D min = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
      Point3D max = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 10.0);
      max.add(min);
      Point3D center = new Point3D();
      center.add(min, max);
      center.scale(0.5);
      Vector3D halfSize = new Vector3D();
      halfSize.sub(max, min);
      halfSize.scale(0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, halfSize);
      EuclidCoreTestTools.assertTuple3DEquals(min, boundingBox.getMinPoint(), EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(max, boundingBox.getMaxPoint(), EPSILON);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox3D boundingBoxOne = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         BoundingBox3D boundingBoxTwo = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         BoundingBox3D expected = new BoundingBox3D();
         expected.combine(boundingBoxOne, boundingBoxTwo);
         BoundingBox3D actual = BoundingBox3D.union(boundingBoxOne, boundingBoxTwo);

         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }
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
         boundingBox.set(new Point3D(1.0, 0.0, 0.0), new Vector3D(-1.0, 0.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(new Point3D(1.0, 0.0, 0.0), new Vector3D(0.0, -1.0, 0.0));
         fail("Should have thrown a " + BoundingBoxException.class.getSimpleName());
      }
      catch (BoundingBoxException e)
      {
         // good
      }
      try
      {
         boundingBox.set(new Point3D(1.0, 0.0, 0.0), new Vector3D(0.0, 0.0, -1.0));
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
         assertTrue(combined.isInsideInclusive(boundingBoxOne.getMinPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxOne.getMaxPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxTwo.getMinPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxTwo.getMaxPoint()));

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
         assertTrue(combined.isInsideInclusive(boundingBoxOne.getMinPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxOne.getMaxPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxTwo.getMinPoint()));
         assertTrue(combined.isInsideInclusive(boundingBoxTwo.getMaxPoint()));

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

         // Simple tests to verify the exclusive feature
         assertFalse(boundingBoxEpsilon.isInsideEpsilon(boundingBoxEpsilon.getMaxPoint(), 0.0));
         assertFalse(boundingBoxEpsilon.isInsideEpsilon(boundingBoxEpsilon.getMinPoint(), 0.0));
      }
   }

   @Test
   public void testIsXYInsideExclusive() throws Exception
   {
      Random random = new Random(3243L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         Point3DReadOnly minPoint = boundingBox3D.getMinPoint();
         Point3DReadOnly maxPoint = boundingBox3D.getMaxPoint();
         assertFalse(boundingBox3D.isXYInsideExclusive(new Point2D(minPoint.getX(), minPoint.getY())));
         assertFalse(boundingBox3D.isXYInsideExclusive(new Point2D(maxPoint.getX(), maxPoint.getY())));
         assertFalse(boundingBox3D.isXYInsideExclusive(minPoint.getX(), minPoint.getY()));
         assertFalse(boundingBox3D.isXYInsideExclusive(maxPoint.getX(), maxPoint.getY()));

         Point3D alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, Double.MIN_VALUE, 1.0 - 1.0e-15);
         Point2D query2D = new Point2D();
         Point3D query3D = new Point3D();
         boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), 0.0, query3D);
         query2D.set(query3D.getX(), query3D.getY());
         assertTrue(boundingBox3D.isXYInsideExclusive(query2D));
         assertTrue(boundingBox3D.isXYInsideExclusive(query2D.getX(), query2D.getY()));

         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, Double.MIN_VALUE, 1.0 - 1.0e-15);
            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query3D);
            query2D.set(query3D.getX(), query3D.getY());
            assertFalse(boundingBox3D.isXYInsideExclusive(query2D));
            assertFalse(boundingBox3D.isXYInsideExclusive(query2D.getX(), query2D.getY()));

            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query3D);
            query2D.set(query3D.getX(), query3D.getY());
            assertFalse(boundingBox3D.isXYInsideExclusive(query2D));
            assertFalse(boundingBox3D.isXYInsideExclusive(query2D.getX(), query2D.getY()));

            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, Double.MIN_VALUE, 1.0 - 1.0e-15);
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query3D);
            query2D.set(query3D.getX(), query3D.getY());
            query2D.setElement(axisIndex, minPoint.getElement(axisIndex));
            assertFalse(boundingBox3D.isXYInsideExclusive(query2D));
            assertFalse(boundingBox3D.isXYInsideExclusive(query2D.getX(), query2D.getY()));

            query2D.setElement(axisIndex, maxPoint.getElement(axisIndex));
            assertFalse(boundingBox3D.isXYInsideExclusive(query2D));
            assertFalse(boundingBox3D.isXYInsideExclusive(query2D.getX(), query2D.getY()));
         }
      }
   }

   @Test
   public void testIsXYInsideInclusive() throws Exception
   {
      Random random = new Random(3243L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         Point3DReadOnly minPoint = boundingBox3D.getMinPoint();
         Point3DReadOnly maxPoint = boundingBox3D.getMaxPoint();
         assertTrue(boundingBox3D.isXYInsideInclusive(new Point2D(minPoint.getX(), minPoint.getY())));
         assertTrue(boundingBox3D.isXYInsideInclusive(new Point2D(maxPoint.getX(), maxPoint.getY())));
         assertTrue(boundingBox3D.isXYInsideInclusive(minPoint.getX(), minPoint.getY()));
         assertTrue(boundingBox3D.isXYInsideInclusive(maxPoint.getX(), maxPoint.getY()));

         Point3D alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
         Point3D query = new Point3D();
         boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
         assertTrue(boundingBox3D.isInsideInclusive(query));
         assertTrue(boundingBox3D.isInsideInclusive(query.getX(), query.getY(), query.getZ()));

         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertFalse(boundingBox3D.isXYInsideInclusive(new Point2D(query.getX(), query.getY())));
            assertFalse(boundingBox3D.isXYInsideInclusive(query.getX(), query.getY()));

            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertFalse(boundingBox3D.isXYInsideInclusive(new Point2D(query.getX(), query.getY())));
            assertFalse(boundingBox3D.isXYInsideInclusive(query.getX(), query.getY()));

            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
            boundingBox3D.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            query.setElement(axisIndex, minPoint.getElement(axisIndex));
            assertTrue(boundingBox3D.isXYInsideInclusive(new Point2D(query.getX(), query.getY())));
            assertTrue(boundingBox3D.isXYInsideInclusive(query.getX(), query.getY()));
            query.setElement(axisIndex, maxPoint.getElement(axisIndex));
            assertTrue(boundingBox3D.isXYInsideInclusive(new Point2D(query.getX(), query.getY())));
            assertTrue(boundingBox3D.isXYInsideInclusive(query.getX(), query.getY()));
         }
      }
   }

   @Test
   public void testIsXYInsideEpsilon() throws Exception
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
         assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(minPoint.getX(), minPoint.getY())),
                      boundingBoxEpsilon.isXYInsideEpsilon(new Point2D(minPoint.getX(), minPoint.getY()), epsilon));
         assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(maxPoint.getX(), maxPoint.getY())),
                      boundingBoxEpsilon.isXYInsideEpsilon(new Point2D(maxPoint.getX(), maxPoint.getY()), epsilon));
         assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(minPoint.getX(), minPoint.getY())),
                      boundingBoxEpsilon.isXYInsideEpsilon(minPoint.getX(), minPoint.getY(), epsilon));
         assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(maxPoint.getX(), maxPoint.getY())),
                      boundingBoxEpsilon.isXYInsideEpsilon(maxPoint.getX(), maxPoint.getY(), epsilon));

         Point3D alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
         Point3D query = new Point3D();
         boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
         assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                      boundingBoxEpsilon.isXYInsideEpsilon(new Point2D(query.getX(), query.getY()), epsilon));
         assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                      boundingBoxEpsilon.isXYInsideEpsilon(query.getX(), query.getY(), epsilon));

         for (int axisIndex = 0; axisIndex < 2; axisIndex++)
         {
            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, -10.0, 0.0));
            boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                         boundingBoxEpsilon.isXYInsideEpsilon(new Point2D(query.getX(), query.getY()), epsilon));
            assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                         boundingBoxEpsilon.isXYInsideEpsilon(query.getX(), query.getY(), epsilon));

            alpha.setElement(axisIndex, EuclidCoreRandomTools.generateRandomDouble(random, 1.0, 10.0));
            boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                         boundingBoxEpsilon.isXYInsideEpsilon(new Point2D(query.getX(), query.getY()), epsilon));
            assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                         boundingBoxEpsilon.isXYInsideEpsilon(query.getX(), query.getY(), epsilon));

            alpha = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.0, 1.0);
            boundingBoxEpsilon.getPointGivenParameters(alpha.getX(), alpha.getY(), alpha.getZ(), query);
            query.setElement(axisIndex, minPoint.getElement(axisIndex));
            assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                         boundingBoxEpsilon.isXYInsideEpsilon(new Point2D(query.getX(), query.getY()), epsilon));
            assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                         boundingBoxEpsilon.isXYInsideEpsilon(query.getX(), query.getY(), epsilon));

            query.setElement(axisIndex, maxPoint.getElement(axisIndex));
            assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                         boundingBoxEpsilon.isXYInsideEpsilon(new Point2D(query.getX(), query.getY()), epsilon));
            assertEquals(boundingBoxExclusive.isXYInsideExclusive(new Point2D(query.getX(), query.getY())),
                         boundingBoxEpsilon.isXYInsideEpsilon(query.getX(), query.getY(), epsilon));
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

         for (double xSign = -1.0; xSign <= 1.0; xSign += 1.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 1.0)
            {
               for (double zSign = -1.0; zSign <= 1.0; zSign += 1.0)
               {
                  Vector3D shift = new Vector3D();
                  shift.add(queryHalfSize, halfSize);
                  shift.scale(xSign, ySign, zSign);

                  queryCenter.scaleAdd(0.999, shift, center);
                  query.set(queryCenter, queryHalfSize);
                  assertTrue(boundingBox3D.intersectsExclusive(query));

                  if (Math.abs(xSign) + Math.abs(ySign) + Math.abs(zSign) != 0.0)
                  {
                     queryCenter.scaleAdd(1.0001, shift, center);
                     query.set(queryCenter, queryHalfSize);
                     assertFalse(boundingBox3D.intersectsExclusive(query));
                  }
               }
            }
         }

         // Simple tests to verify the exclusive feature
         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(-1.0, 0.0, 0.0, 0.0, 1.0, 1.0);
         assertFalse(boundingBox3D.intersectsExclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, -1.0, 0.0, 1.0, 0.0, 1.0);
         assertFalse(boundingBox3D.intersectsExclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, 0.0, -1.0, 1.0, 1.0, 0.0);
         assertFalse(boundingBox3D.intersectsExclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);
         assertFalse(boundingBox3D.intersectsExclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
         assertFalse(boundingBox3D.intersectsExclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, 0.0, 1.0, 1.0, 1.0, 2.0);
         assertFalse(boundingBox3D.intersectsExclusive(query));
      }
   }

   @Test
   public void testIntersectsInclusive() throws Exception
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
         assertTrue(boundingBox3D.intersectsInclusive(query));

         for (double xSign = -1.0; xSign <= 1.0; xSign += 1.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 1.0)
            {
               for (double zSign = -1.0; zSign <= 1.0; zSign += 1.0)
               {
                  Vector3D shift = new Vector3D();
                  shift.add(queryHalfSize, halfSize);
                  shift.scale(xSign, ySign, zSign);

                  queryCenter.scaleAdd(0.999, shift, center);
                  query.set(queryCenter, queryHalfSize);
                  assertTrue(boundingBox3D.intersectsInclusive(query));

                  if (Math.abs(xSign) + Math.abs(ySign) + Math.abs(zSign) != 0.0)
                  {
                     queryCenter.scaleAdd(1.0001, shift, center);
                     query.set(queryCenter, queryHalfSize);
                     assertFalse(boundingBox3D.intersectsInclusive(query));
                  }
               }
            }
         }

         // Simple tests to verify the inclusive feature
         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(-1.0, 0.0, 0.0, 0.0, 1.0, 1.0);
         assertTrue(boundingBox3D.intersectsInclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, -1.0, 0.0, 1.0, 0.0, 1.0);
         assertTrue(boundingBox3D.intersectsInclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, 0.0, -1.0, 1.0, 1.0, 0.0);
         assertTrue(boundingBox3D.intersectsInclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);
         assertTrue(boundingBox3D.intersectsInclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
         assertTrue(boundingBox3D.intersectsInclusive(query));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, 0.0, 1.0, 1.0, 1.0, 2.0);
         assertTrue(boundingBox3D.intersectsInclusive(query));
      }
   }

   @Test
   public void testIntersectsEpsilon() throws Exception
   {
      Random random = new Random(34545L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = EuclidCoreRandomTools.generateRandomDouble(random, 1.0);

         Point3D center = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Vector3D halfSize = EuclidCoreRandomTools.generateRandomVector3D(random, 2.0 * Math.abs(epsilon), 10.0);
         BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, halfSize);

         Point3D queryCenter = new Point3D();
         Vector3D queryHalfSize = EuclidCoreRandomTools.generateRandomVector3D(random, 2.0 * Math.abs(epsilon), 10.0);
         BoundingBox3D query = new BoundingBox3D();

         double xParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         double yParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         double zParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         boundingBox3D.getPointGivenParameters(xParameter, yParameter, zParameter, queryCenter);
         query.set(queryCenter, queryHalfSize);
         assertTrue(boundingBox3D.intersectsEpsilon(query, epsilon));

         for (double xSign = -1.0; xSign <= 1.0; xSign += 1.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 1.0)
            {
               for (double zSign = -1.0; zSign <= 1.0; zSign += 1.0)
               {
                  Vector3D shift = new Vector3D();
                  shift.add(queryHalfSize, halfSize);
                  shift.add(epsilon, epsilon, epsilon);
                  shift.scale(xSign, ySign, zSign);

                  queryCenter.scaleAdd(0.999, shift, center);
                  query.set(queryCenter, queryHalfSize);
                  assertTrue(boundingBox3D.intersectsEpsilon(query, epsilon));

                  if (Math.abs(xSign) + Math.abs(ySign) + Math.abs(zSign) != 0.0)
                  {
                     queryCenter.scaleAdd(1.0001, shift, center);
                     query.set(queryCenter, queryHalfSize);
                     assertFalse(boundingBox3D.intersectsEpsilon(query, epsilon));
                  }
               }
            }
         }

         // Simple tests to verify the exclusive feature
         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(-1.0, 0.0, 0.0, 0.0, 1.0, 1.0);
         assertFalse(boundingBox3D.intersectsEpsilon(query, 0.0));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, -1.0, 0.0, 1.0, 0.0, 1.0);
         assertFalse(boundingBox3D.intersectsEpsilon(query, 0.0));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, 0.0, -1.0, 1.0, 1.0, 0.0);
         assertFalse(boundingBox3D.intersectsEpsilon(query, 0.0));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(1.0, 0.0, 0.0, 2.0, 1.0, 1.0);
         assertFalse(boundingBox3D.intersectsEpsilon(query, 0.0));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, 1.0, 0.0, 1.0, 2.0, 1.0);
         assertFalse(boundingBox3D.intersectsEpsilon(query, 0.0));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query.set(0.0, 0.0, 1.0, 1.0, 1.0, 2.0);
         assertFalse(boundingBox3D.intersectsEpsilon(query, 0.0));
      }
   }

   @Test
   public void testIntersectsExclusiveInXYPlane() throws Exception
   {
      Random random = new Random(34545L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D center3D = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Point2D center2D = new Point2D(center3D.getX(), center3D.getY());
         Vector3D halfSize3D = EuclidCoreRandomTools.generateRandomVector3D(random, 0.0, 10.0);
         BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(center3D, halfSize3D);

         Point2D queryCenter2D = new Point2D();
         Point3D queryCenter3D = new Point3D();
         Vector2D queryHalfSize2D = EuclidCoreRandomTools.generateRandomVector2D(random, 0.0, 10.0);
         BoundingBox2D query2D = new BoundingBox2D();

         double xParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         double yParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         boundingBox3D.getPointGivenParameters(xParameter, yParameter, 0.0, queryCenter3D);
         queryCenter2D.set(queryCenter3D.getX(), queryCenter3D.getY());
         query2D.set(queryCenter2D, queryHalfSize2D);
         assertTrue(boundingBox3D.intersectsExclusiveInXYPlane(query2D));

         for (double xSign = -1.0; xSign <= 1.0; xSign += 1.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 1.0)
            {
               Vector2D shift2D = new Vector2D(queryHalfSize2D);
               shift2D.add(halfSize3D.getX(), halfSize3D.getY());
               shift2D.scale(xSign, ySign);

               queryCenter2D.scaleAdd(0.999, shift2D, center2D);
               query2D.set(queryCenter2D, queryHalfSize2D);
               assertTrue(boundingBox3D.intersectsExclusiveInXYPlane(query2D));

               if (Math.abs(xSign) + Math.abs(ySign) != 0.0)
               {
                  queryCenter2D.scaleAdd(1.0001, shift2D, center2D);
                  query2D.set(queryCenter2D, queryHalfSize2D);
                  assertFalse(boundingBox3D.intersectsExclusiveInXYPlane(query2D));
               }
            }
         }

         // Simple tests to verify the exclusive feature
         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(-1.0, 0.0, 0.0, 1.0);
         assertFalse(boundingBox3D.intersectsExclusiveInXYPlane(query2D));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(0.0, -1.0, 1.0, 0.0);
         assertFalse(boundingBox3D.intersectsExclusiveInXYPlane(query2D));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(1.0, 0.0, 2.0, 1.0);
         assertFalse(boundingBox3D.intersectsExclusiveInXYPlane(query2D));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(0.0, 1.0, 1.0, 2.0);
         assertFalse(boundingBox3D.intersectsExclusiveInXYPlane(query2D));
      }
   }

   @Test
   public void testIntersectsInclusiveInXYPlane() throws Exception
   {
      Random random = new Random(34545L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D center3D = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Point2D center2D = new Point2D(center3D.getX(), center3D.getY());
         Vector3D halfSize3D = EuclidCoreRandomTools.generateRandomVector3D(random, 0.0, 10.0);
         BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(center3D, halfSize3D);

         Point2D queryCenter2D = new Point2D();
         Point3D queryCenter3D = new Point3D();
         Vector2D queryHalfSize2D = EuclidCoreRandomTools.generateRandomVector2D(random, 0.0, 10.0);
         BoundingBox2D query2D = new BoundingBox2D();

         double xParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         double yParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         boundingBox3D.getPointGivenParameters(xParameter, yParameter, 0.0, queryCenter3D);
         queryCenter2D.set(queryCenter3D.getX(), queryCenter3D.getY());
         query2D.set(queryCenter2D, queryHalfSize2D);
         assertTrue(boundingBox3D.intersectsExclusiveInXYPlane(query2D));

         for (double xSign = -1.0; xSign <= 1.0; xSign += 1.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 1.0)
            {
               Vector2D shift = new Vector2D(queryHalfSize2D);
               shift.add(halfSize3D.getX(), halfSize3D.getY());
               shift.scale(xSign, ySign);

               queryCenter2D.scaleAdd(0.999, shift, center2D);
               query2D.set(queryCenter2D, queryHalfSize2D);
               assertTrue(boundingBox3D.intersectsInclusiveInXYPlane(query2D));

               if (Math.abs(xSign) + Math.abs(ySign) != 0.0)
               {
                  queryCenter2D.scaleAdd(1.0001, shift, center2D);
                  query2D.set(queryCenter2D, queryHalfSize2D);
                  assertFalse(boundingBox3D.intersectsInclusiveInXYPlane(query2D));
               }
            }
         }

         // Simple tests to verify the inclusive feature
         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(-1.0, 0.0, 0.0, 1.0);
         assertTrue(boundingBox3D.intersectsInclusiveInXYPlane(query2D));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(0.0, -1.0, 1.0, 0.0);
         assertTrue(boundingBox3D.intersectsInclusiveInXYPlane(query2D));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(1.0, 0.0, 2.0, 1.0);
         assertTrue(boundingBox3D.intersectsInclusiveInXYPlane(query2D));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(0.0, 1.0, 1.0, 2.0);
         assertTrue(boundingBox3D.intersectsInclusiveInXYPlane(query2D));
      }
   }

   @Test
   public void testIntersectsEpsilonInXYPlane() throws Exception
   {
      Random random = new Random(34545L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = EuclidCoreRandomTools.generateRandomDouble(random, 1.0);

         Point3D center3D = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         Point2D center2D = new Point2D(center3D.getX(), center3D.getY());
         Vector3D halfSize3D = EuclidCoreRandomTools.generateRandomVector3D(random, 2.0 * Math.abs(epsilon), 10.0);
         BoundingBox3D boundingBox3D = BoundingBox3D.createUsingCenterAndPlusMinusVector(center3D, halfSize3D);

         Point2D queryCenter2D = new Point2D();
         Point3D queryCenter3D = new Point3D();
         Vector2D queryHalfSize2D = EuclidCoreRandomTools.generateRandomVector2D(random, 2.0 * Math.abs(epsilon), 10.0);
         BoundingBox2D query2D = new BoundingBox2D();

         double xParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         double yParameter = EuclidCoreRandomTools.generateRandomDouble(random, 0.0, 1.0);
         boundingBox3D.getPointGivenParameters(xParameter, yParameter, 0.0, queryCenter3D);
         queryCenter2D.set(queryCenter3D.getX(), queryCenter3D.getY());
         query2D.set(queryCenter2D, queryHalfSize2D);
         assertTrue(boundingBox3D.intersectsEpsilonInXYPlane(query2D, epsilon));

         for (double xSign = -1.0; xSign <= 1.0; xSign += 1.0)
         {
            for (double ySign = -1.0; ySign <= 1.0; ySign += 1.0)
            {
               Vector2D shift = new Vector2D(queryHalfSize2D);
               shift.add(halfSize3D.getX(), halfSize3D.getY());
               shift.add(epsilon, epsilon);
               shift.scale(xSign, ySign);

               queryCenter2D.scaleAdd(0.999, shift, center2D);
               query2D.set(queryCenter2D, queryHalfSize2D);
               assertTrue(boundingBox3D.intersectsEpsilonInXYPlane(query2D, epsilon));

               if (Math.abs(xSign) + Math.abs(ySign) != 0.0)
               {
                  queryCenter2D.scaleAdd(1.0001, shift, center2D);
                  query2D.set(queryCenter2D, queryHalfSize2D);
                  assertFalse(boundingBox3D.intersectsEpsilonInXYPlane(query2D, epsilon));
               }
            }
         }

         // Simple tests to verify the exclusive feature
         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(-1.0, 0.0, 0.0, 1.0);
         assertFalse(boundingBox3D.intersectsEpsilonInXYPlane(query2D, 0.0));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(0.0, -1.0, 1.0, 0.0);
         assertFalse(boundingBox3D.intersectsEpsilonInXYPlane(query2D, 0.0));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(1.0, 0.0, 2.0, 1.0);
         assertFalse(boundingBox3D.intersectsEpsilonInXYPlane(query2D, 0.0));

         boundingBox3D.set(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
         query2D.set(0.0, 1.0, 1.0, 2.0);
         assertFalse(boundingBox3D.intersectsEpsilonInXYPlane(query2D, 0.0));
      }
   }

   @Test
   public void testDoesIntersectWithLine3D() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line3D = EuclidGeometryRandomTools.generateRandomLine3D(random, 10.0);
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         boolean expected = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBox3D.getMinPoint(), boundingBox3D.getMaxPoint(),
                                                                                          line3D.getPoint(), line3D.getDirection(), null, null) != 0;
         boolean actual = boundingBox3D.doesIntersectWithLine3D(line3D);
         assertEquals(expected, actual);

         actual = boundingBox3D.doesIntersectWithLine3D(line3D.getPoint(), line3D.getDirection());
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testDoesIntersectWithLineSegment3D() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         boolean expected = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBox3D.getMinPoint(), boundingBox3D.getMaxPoint(),
                                                                                                 lineSegment3D.getFirstEndpoint(),
                                                                                                 lineSegment3D.getSecondEndpoint(), null, null) != 0;
         boolean actual = boundingBox3D.doesIntersectWithLineSegment3D(lineSegment3D);
         assertEquals(expected, actual);

         actual = boundingBox3D.doesIntersectWithLineSegment3D(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testDoesIntersectWithRay3D() throws Exception
   {
      Random random = new Random(23423L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D ray3D = EuclidGeometryRandomTools.generateRandomLine3D(random, 10.0);
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         boolean expected = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBox3D.getMinPoint(), boundingBox3D.getMaxPoint(),
                                                                                         ray3D.getPoint(), ray3D.getDirection(), null, null) != 0;
         boolean actual = boundingBox3D.doesIntersectWithRay3D(ray3D.getPoint(), ray3D.getDirection());
         assertEquals(expected, actual);
      }
   }

   @Test
   public void testintersectionWithLine3D() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D line3D = EuclidGeometryRandomTools.generateRandomLine3D(random, 10.0);
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);

         int expectedN, actualN;
         Point3D expectedFirstIntersection = new Point3D();
         Point3D expectedSecondIntersection = new Point3D();
         Point3D actualFirstIntersection = new Point3D();
         Point3D actualSecondIntersection = new Point3D();
         expectedN = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(boundingBox3D.getMinPoint(), boundingBox3D.getMaxPoint(), line3D.getPoint(),
                                                                                   line3D.getDirection(), expectedFirstIntersection,
                                                                                   expectedSecondIntersection);
         actualN = boundingBox3D.intersectionWithLine3D(line3D.getPoint(), line3D.getDirection(), actualFirstIntersection, actualSecondIntersection);

         assertEquals(expectedN, actualN);
         if (expectedN == 0)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedFirstIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedFirstIntersection, actualFirstIntersection, EPSILON);
         }

         if (expectedN <= 1)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedSecondIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedSecondIntersection, actualSecondIntersection, EPSILON);
         }

         actualFirstIntersection.setToZero();
         actualSecondIntersection.setToZero();

         actualN = boundingBox3D.intersectionWithLine3D(line3D, actualFirstIntersection, actualSecondIntersection);

         assertEquals(expectedN, actualN);
         if (expectedN == 0)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedFirstIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedFirstIntersection, actualFirstIntersection, EPSILON);
         }

         if (expectedN <= 1)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedSecondIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedSecondIntersection, actualSecondIntersection, EPSILON);
         }
      }
   }

   @Test
   public void testIntersectionWithLineSegment3D() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         LineSegment3D lineSegment3D = EuclidGeometryRandomTools.generateRandomLineSegment3D(random, 10.0);
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);

         int expectedN, actualN;
         Point3D expectedFirstIntersection = new Point3D();
         Point3D expectedSecondIntersection = new Point3D();
         Point3D actualFirstIntersection = new Point3D();
         Point3D actualSecondIntersection = new Point3D();
         expectedN = EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D(boundingBox3D.getMinPoint(), boundingBox3D.getMaxPoint(),
                                                                                          lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint(),
                                                                                          expectedFirstIntersection, expectedSecondIntersection);
         actualN = boundingBox3D.intersectionWithLineSegment3D(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint(), actualFirstIntersection,
                                                               actualSecondIntersection);

         assertEquals(expectedN, actualN);
         if (expectedN == 0)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedFirstIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedFirstIntersection, actualFirstIntersection, EPSILON);
         }

         if (expectedN <= 1)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedSecondIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedSecondIntersection, actualSecondIntersection, EPSILON);
         }

         actualFirstIntersection.setToZero();
         actualSecondIntersection.setToZero();

         actualN = boundingBox3D.intersectionWithLineSegment3D(lineSegment3D, actualFirstIntersection, actualSecondIntersection);

         assertEquals(expectedN, actualN);
         if (expectedN == 0)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedFirstIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedFirstIntersection, actualFirstIntersection, EPSILON);
         }

         if (expectedN <= 1)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedSecondIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedSecondIntersection, actualSecondIntersection, EPSILON);
         }
      }
   }

   @Test
   public void testIntersectionWithRay3D() throws Exception
   {
      Random random = new Random(435345L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Line3D ray3D = EuclidGeometryRandomTools.generateRandomLine3D(random, 10.0);
         BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);

         int expectedN, actualN;
         Point3D expectedFirstIntersection = new Point3D();
         Point3D expectedSecondIntersection = new Point3D();
         Point3D actualFirstIntersection = new Point3D();
         Point3D actualSecondIntersection = new Point3D();
         expectedN = EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D(boundingBox3D.getMinPoint(), boundingBox3D.getMaxPoint(), ray3D.getPoint(),
                                                                                  ray3D.getDirection(), expectedFirstIntersection, expectedSecondIntersection);
         actualN = boundingBox3D.intersectionWithRay3D(ray3D.getPoint(), ray3D.getDirection(), actualFirstIntersection, actualSecondIntersection);

         assertEquals(expectedN, actualN);
         if (expectedN == 0)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedFirstIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualFirstIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedFirstIntersection, actualFirstIntersection, EPSILON);
         }

         if (expectedN <= 1)
         {
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(expectedSecondIntersection);
            EuclidCoreTestTools.assertTuple3DContainsOnlyNaN(actualSecondIntersection);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DEquals(expectedSecondIntersection, actualSecondIntersection, EPSILON);
         }
      }
   }

   @Test
   public void testUpdateToIncludePoint() throws Exception
   {
      Random random = new Random(234234L);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test updateToIncludePoint(Point3DReadOnly point)
         BoundingBox3D original = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         BoundingBox3D extended = new BoundingBox3D(original);

         Point3D point = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         extended.updateToIncludePoint(point);

         if (original.isInsideInclusive(point))
         { // Make sure calling it did not change the bounding box
            assertEquals(original, extended);
         }
         else
         { // If a bound has changed, it must be set to the point coordinate.
            assertTrue(extended.isInsideInclusive(point));

            Point3DReadOnly originalMin = original.getMinPoint();
            Point3DReadOnly extendedMin = extended.getMinPoint();

            for (int axis = 0; axis < 3; axis++)
            {
               boolean hasNotChanged = originalMin.getElement(axis) == extendedMin.getElement(axis);
               boolean hasGrownToPoint = point.getElement(axis) == extendedMin.getElement(axis);
               assertTrue(hasNotChanged || hasGrownToPoint);
            }

            Point3DReadOnly originalMax = original.getMaxPoint();
            Point3DReadOnly extendedMax = extended.getMaxPoint();

            for (int axis = 0; axis < 3; axis++)
            {
               boolean hasNotChanged = originalMax.getElement(axis) == extendedMax.getElement(axis);
               boolean hasGrownToPoint = point.getElement(axis) == extendedMax.getElement(axis);
               assertTrue(hasNotChanged || hasGrownToPoint);
            }
         }
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test updateToIncludePoint(Point3DReadOnly point)
         BoundingBox3D original = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
         BoundingBox3D extended = new BoundingBox3D(original);

         Point3D point = EuclidCoreRandomTools.generateRandomPoint3D(random, 10.0);
         extended.updateToIncludePoint(point.getX(), point.getY(), point.getZ());

         if (original.isInsideInclusive(point))
         { // Make sure calling it did not change the bounding box
            assertEquals(original, extended);
         }
         else
         { // If a bound has changed, it must be set to the point coordinate.
            assertTrue(extended.isInsideInclusive(point));

            Point3DReadOnly originalMin = original.getMinPoint();
            Point3DReadOnly extendedMin = extended.getMinPoint();

            for (int axis = 0; axis < 3; axis++)
            {
               boolean hasNotChanged = originalMin.getElement(axis) == extendedMin.getElement(axis);
               boolean hasGrownToPoint = point.getElement(axis) == extendedMin.getElement(axis);
               assertTrue(hasNotChanged || hasGrownToPoint);
            }

            Point3DReadOnly originalMax = original.getMaxPoint();
            Point3DReadOnly extendedMax = extended.getMaxPoint();

            for (int axis = 0; axis < 3; axis++)
            {
               boolean hasNotChanged = originalMax.getElement(axis) == extendedMax.getElement(axis);
               boolean hasGrownToPoint = point.getElement(axis) == extendedMax.getElement(axis);
               assertTrue(hasNotChanged || hasGrownToPoint);
            }
         }
      }
   }

   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(3242L);

      BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
      Point3DReadOnly min = boundingBox3D.getMinPoint();
      Point3DReadOnly max = boundingBox3D.getMaxPoint();

      {
         Point3D actualMin = new Point3D();
         Point3D actualMax = new Point3D();
         boundingBox3D.getMinPoint(actualMin);
         boundingBox3D.getMaxPoint(actualMax);
         assertEquals(min, actualMin);
         assertEquals(max, actualMax);
      }

      {
         double[] actualMin = new double[3];
         double[] actualMax = new double[3];
         boundingBox3D.getMinPoint(actualMin);
         boundingBox3D.getMaxPoint(actualMax);
         assertEquals(min, new Point3D(actualMin));
         assertEquals(max, new Point3D(actualMax));
      }

      {
         assertTrue(min.getX() == boundingBox3D.getMinX());
         assertTrue(min.getY() == boundingBox3D.getMinY());
         assertTrue(min.getZ() == boundingBox3D.getMinZ());
         assertTrue(max.getX() == boundingBox3D.getMaxX());
         assertTrue(max.getY() == boundingBox3D.getMaxY());
         assertTrue(max.getZ() == boundingBox3D.getMaxZ());
      }
   }

   @Test
   public void testEpsilonEquals() throws Exception
   {
      Random random = new Random(234234L);
      double epsilon = random.nextDouble();
      BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
      double minX = boundingBox3D.getMinX();
      double minY = boundingBox3D.getMinY();
      double minZ = boundingBox3D.getMinZ();
      double maxX = boundingBox3D.getMaxX();
      double maxY = boundingBox3D.getMaxY();
      double maxZ = boundingBox3D.getMaxZ();

      double small = 0.999 * epsilon;
      double big = 1.001 * epsilon;

      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX + small, minY, minZ, maxX, maxY, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY + small, minZ, maxX, maxY, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ + small, maxX, maxY, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX + small, maxY, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX, maxY + small, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ + small), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX - small, minY, minZ, maxX, maxY, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY - small, minZ, maxX, maxY, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ - small, maxX, maxY, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX - small, maxY, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX, maxY - small, maxZ), epsilon));
      assertTrue(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ - small), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX + big, minY, minZ, maxX, maxY, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY + big, minZ, maxX, maxY, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ + big, maxX, maxY, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX + big, maxY, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX, maxY + big, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ + big), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX - big, minY, minZ, maxX, maxY, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY - big, minZ, maxX, maxY, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ - big, maxX, maxY, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX - big, maxY, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX, maxY - big, maxZ), epsilon));
      assertFalse(boundingBox3D.epsilonEquals(new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ - big), epsilon));
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(234234L);
      BoundingBox3D boundingBox3D = EuclidGeometryRandomTools.generateRandomBoundingBox3D(random, 10.0, 10.0);
      double minX = boundingBox3D.getMinX();
      double minY = boundingBox3D.getMinY();
      double minZ = boundingBox3D.getMinZ();
      double maxX = boundingBox3D.getMaxX();
      double maxY = boundingBox3D.getMaxY();
      double maxZ = boundingBox3D.getMaxZ();

      double smallestEpsilon = 8.8888e-16;

      assertTrue(boundingBox3D.equals(new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ)));
      assertTrue(boundingBox3D.equals((Object) new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ)));
      assertFalse(boundingBox3D.equals(null));
      assertFalse(boundingBox3D.equals(new double[5]));

      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX + smallestEpsilon, minY, minZ, maxX, maxY, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY + smallestEpsilon, minZ, maxX, maxY, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY, minZ + smallestEpsilon, maxX, maxY, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY, minZ, maxX + smallestEpsilon, maxY, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY, minZ, maxX, maxY + smallestEpsilon, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ + smallestEpsilon)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX - smallestEpsilon, minY, minZ, maxX, maxY, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY - smallestEpsilon, minZ, maxX, maxY, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY, minZ - smallestEpsilon, maxX, maxY, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY, minZ, maxX - smallestEpsilon, maxY, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY, minZ, maxX, maxY - smallestEpsilon, maxZ)));
      assertFalse(boundingBox3D.equals(new BoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ - smallestEpsilon)));
   }
   
   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Random random = new Random(987234L);
      BoundingBox3D firstBox, secondBox;
      Point3D firstPoint, secondPoint, thirdPoint;
      double epsilon = 1e-7;
   
      for (int i = 0; i < ITERATIONS; ++i) {
         firstPoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 0.1, 2.5);
         secondPoint = EuclidCoreRandomTools.generateRandomPoint3D(random, 2.5, 5.0);
   
         thirdPoint = new Point3D(firstPoint);
   
         firstBox = new BoundingBox3D(firstPoint, secondPoint);
         secondBox = new BoundingBox3D(thirdPoint, secondPoint);
   
         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));
   
         thirdPoint.setX(firstPoint.getX() + epsilon * 0.99);
         secondBox = new BoundingBox3D(thirdPoint, secondPoint);
         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));
   
         thirdPoint = new Point3D(firstPoint);
         thirdPoint.setY(firstPoint.getY() + epsilon * 0.99);
         secondBox = new BoundingBox3D(thirdPoint, secondPoint);
         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));
   
         thirdPoint = new Point3D(firstPoint);
         thirdPoint.setX(firstPoint.getX() - epsilon * 0.99);
         secondBox = new BoundingBox3D(thirdPoint, secondPoint);
         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));
   
         thirdPoint = new Point3D(firstPoint);
         thirdPoint.setY(firstPoint.getY() - epsilon * 0.99);
         secondBox = new BoundingBox3D(thirdPoint, secondPoint);
         assertTrue(firstBox.geometricallyEquals(secondBox, epsilon));
   
         thirdPoint = new Point3D(firstPoint);
         thirdPoint.setX(firstPoint.getX() + epsilon * 1.01);
         secondBox = new BoundingBox3D(thirdPoint, secondPoint);
         assertFalse(firstBox.geometricallyEquals(secondBox, epsilon));
   
         thirdPoint = new Point3D(firstPoint);
         thirdPoint.setY(firstPoint.getY() + epsilon * 1.01);
         secondBox = new BoundingBox3D(thirdPoint, secondPoint);
         assertFalse(firstBox.geometricallyEquals(secondBox, epsilon));
   
         thirdPoint = new Point3D(firstPoint);
         thirdPoint.setX(firstPoint.getX() - epsilon * 1.01);
         secondBox = new BoundingBox3D(thirdPoint, secondPoint);
         assertFalse(firstBox.geometricallyEquals(secondBox, epsilon));
   
         thirdPoint = new Point3D(firstPoint);
         thirdPoint.setY(firstPoint.getY() - epsilon * 1.01);
         secondBox = new BoundingBox3D(thirdPoint, secondPoint);
         assertFalse(firstBox.geometricallyEquals(secondBox, epsilon));
      }
   }
}
