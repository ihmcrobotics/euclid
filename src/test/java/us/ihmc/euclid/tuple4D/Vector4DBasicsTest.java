package us.ihmc.euclid.tuple4D;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;

public abstract class Vector4DBasicsTest<T extends Vector4DBasics> extends Tuple4DBasicsTest<T>
{
   @Override
   public void testSetDoubles()
   {
      Random random = new Random(621541L);
      T vector = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(double x, double y, double z, double s);
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         vector.set(x, y, z, s);

         assertEquals(vector.getX(), x, getEpsilon());
         assertEquals(vector.getY(), y, getEpsilon());
         assertEquals(vector.getZ(), z, getEpsilon());
         assertEquals(vector.getS(), s, getEpsilon());
      }
   }

   @Test
   public void testSetToZero() throws Exception
   {
      Random random = new Random(621541L);
      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple = createEmptyTuple();
         tuple.setX(random.nextDouble());
         tuple.setY(random.nextDouble());
         tuple.setZ(random.nextDouble());
         tuple.setS(random.nextDouble());

         tuple.setToZero();
         assertTrue(tuple.getX() == 0.0);
         assertTrue(tuple.getY() == 0.0);
         assertTrue(tuple.getZ() == 0.0);
         assertTrue(tuple.getS() == 0.0);
      }
   }

   @Test
   public void testTuple3DSetters() throws Exception
   {
      Random random = new Random(4325234L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple = createEmptyTuple();
         Point3D point3D = EuclidCoreRandomTools.nextPoint3D(random);
         tuple.set(point3D);
         for (int index = 0; index < 3; index++)
            assertEquals(tuple.getElement(index), point3D.getElement(index), getEpsilon());
         assertTrue(tuple.getS() == 1.0);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         T tuple = createEmptyTuple();
         Vector3D vector3D = EuclidCoreRandomTools.nextVector3D(random);
         tuple.set(vector3D);
         for (int index = 0; index < 3; index++)
            assertEquals(tuple.getElement(index), vector3D.getElement(index), getEpsilon());
         assertTrue(tuple.getS() == 0.0);
      }
   }

   @Test
   public void testSettersForIndividualComponents() throws Exception
   {
      Random random = new Random(621541L);
      T vector = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setX(double x)
         double x = random.nextDouble();
         vector.setX(x);
         assertEquals(vector.getX(), x, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setY(double y)
         double y = random.nextDouble();
         vector.setY(y);
         assertEquals(vector.getY(), y, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setZ(double z)
         double z = random.nextDouble();
         vector.setZ(z);
         assertEquals(vector.getZ(), z, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test setS(double s)
         double s = random.nextDouble();
         vector.setS(s);
         assertEquals(vector.getS(), s, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test set(int index, double value)
         try
         {
            vector.setElement(-1, random.nextDouble());
            fail("Should have thrown a IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IndexOutOfBoundsException.");
         }

         try
         {
            vector.setElement(4, random.nextDouble());
            fail("Should have thrown a IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {
            // good
         }
         catch (Exception e)
         {
            fail("Should have thrown a IndexOutOfBoundsException.");
         }

         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         vector.setElement(0, x);
         vector.setElement(1, y);
         vector.setElement(2, z);
         vector.setElement(3, s);

         assertEquals(vector.getX(), x, getEpsilon());
         assertEquals(vector.getY(), y, getEpsilon());
         assertEquals(vector.getZ(), z, getEpsilon());
         assertEquals(vector.getS(), s, getEpsilon());
      }
   }

   @Test
   public void testClip() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMax(double max)
         double max = random.nextDouble();
         tuple1.setX(max + random.nextDouble());
         tuple1.setY(max + random.nextDouble());
         tuple1.setZ(max + random.nextDouble());
         tuple1.setS(max + random.nextDouble());
         tuple1.clipToMax(max);
         assertEquals(tuple1.getX(), max, getEpsilon());
         assertEquals(tuple1.getY(), max, getEpsilon());
         assertEquals(tuple1.getZ(), max, getEpsilon());
         assertEquals(tuple1.getS(), max, getEpsilon());

         max = random.nextDouble();
         tuple1.setX(max - random.nextDouble());
         tuple1.setY(max - random.nextDouble());
         tuple1.setZ(max - random.nextDouble());
         tuple1.setS(max - random.nextDouble());
         tuple1.set(tuple2);
         tuple1.clipToMax(max);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMax(double max, TupleBasics other)
         double max = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple1.setS(random.nextDouble());
         tuple2.setX(max + random.nextDouble());
         tuple2.setY(max + random.nextDouble());
         tuple2.setZ(max + random.nextDouble());
         tuple2.setS(max + random.nextDouble());
         tuple1.setAndClipToMax(max, tuple2);
         assertEquals(tuple1.getX(), max, getEpsilon());
         assertEquals(tuple1.getY(), max, getEpsilon());
         assertEquals(tuple1.getZ(), max, getEpsilon());
         assertEquals(tuple1.getS(), max, getEpsilon());

         max = random.nextDouble();
         tuple2.setX(max - random.nextDouble());
         tuple2.setY(max - random.nextDouble());
         tuple2.setZ(max - random.nextDouble());
         tuple2.setS(max - random.nextDouble());
         tuple1.setAndClipToMax(max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMin(double min)
         double min = random.nextDouble();
         tuple1.setX(min - random.nextDouble());
         tuple1.setY(min - random.nextDouble());
         tuple1.setZ(min - random.nextDouble());
         tuple1.setS(min - random.nextDouble());
         tuple1.clipToMin(min);
         assertEquals(tuple1.getX(), min, getEpsilon());
         assertEquals(tuple1.getY(), min, getEpsilon());
         assertEquals(tuple1.getZ(), min, getEpsilon());
         assertEquals(tuple1.getS(), min, getEpsilon());

         min = random.nextDouble();
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple2.setS(min + random.nextDouble());
         tuple1.set(tuple2);
         tuple1.clipToMin(min);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMin(double min, TupleBasics other)
         double min = random.nextDouble();
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple1.setS(random.nextDouble());
         tuple2.setX(min - random.nextDouble());
         tuple2.setY(min - random.nextDouble());
         tuple2.setZ(min - random.nextDouble());
         tuple2.setS(min - random.nextDouble());
         tuple1.setAndClipToMin(min, tuple2);
         assertEquals(tuple1.getX(), min, getEpsilon());
         assertEquals(tuple1.getY(), min, getEpsilon());
         assertEquals(tuple1.getZ(), min, getEpsilon());
         assertEquals(tuple1.getS(), min, getEpsilon());

         min = random.nextDouble();
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple2.setS(min + random.nextDouble());
         tuple1.setAndClipToMin(min, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMinMax(double min, double max)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         tuple1.setX(min - random.nextDouble());
         tuple1.setY(min - random.nextDouble());
         tuple1.setZ(min - random.nextDouble());
         tuple1.setS(min - random.nextDouble());
         tuple1.clipToMinMax(min, max);
         assertEquals(tuple1.getX(), min, getEpsilon());
         assertEquals(tuple1.getY(), min, getEpsilon());
         assertEquals(tuple1.getS(), min, getEpsilon());

         min = random.nextDouble() - 0.5;
         max = random.nextDouble() + 0.5;
         tuple1.setX(max + random.nextDouble());
         tuple1.setY(max + random.nextDouble());
         tuple1.setZ(max + random.nextDouble());
         tuple1.setS(max + random.nextDouble());
         tuple1.clipToMinMax(min, max);
         assertEquals(tuple1.getX(), max, getEpsilon());
         assertEquals(tuple1.getY(), max, getEpsilon());
         assertEquals(tuple1.getZ(), max, getEpsilon());
         assertEquals(tuple1.getS(), max, getEpsilon());

         min = random.nextDouble() - 1.0;
         max = random.nextDouble() + 1.0;
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple2.setS(min + random.nextDouble());
         tuple1.set(tuple2);
         tuple1.clipToMinMax(min, max);
         assertTrue(tuple1.equals(tuple2));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test clipToMinMax(double min, double max, TupleBasics other)
         double min = random.nextDouble() - 0.5;
         double max = random.nextDouble() + 0.5;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple1.setS(random.nextDouble());
         tuple2.setX(min - random.nextDouble());
         tuple2.setY(min - random.nextDouble());
         tuple2.setZ(min - random.nextDouble());
         tuple2.setS(min - random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertEquals(tuple1.getX(), min, getEpsilon());
         assertEquals(tuple1.getY(), min, getEpsilon());
         assertEquals(tuple1.getZ(), min, getEpsilon());
         assertEquals(tuple1.getS(), min, getEpsilon());

         min = random.nextDouble() - 0.5;
         max = random.nextDouble() + 0.5;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple1.setS(random.nextDouble());
         tuple2.setX(max + random.nextDouble());
         tuple2.setY(max + random.nextDouble());
         tuple2.setZ(max + random.nextDouble());
         tuple2.setS(max + random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertEquals(tuple1.getX(), max, getEpsilon());
         assertEquals(tuple1.getY(), max, getEpsilon());
         assertEquals(tuple1.getZ(), max, getEpsilon());
         assertEquals(tuple1.getS(), max, getEpsilon());

         min = random.nextDouble() - 1.0;
         max = random.nextDouble() + 1.0;
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple1.setS(random.nextDouble());
         tuple2.setX(min + random.nextDouble());
         tuple2.setY(min + random.nextDouble());
         tuple2.setZ(min + random.nextDouble());
         tuple2.setS(min + random.nextDouble());
         tuple1.setAndClipToMinMax(min, max, tuple2);
         assertTrue(tuple1.equals(tuple2));
      }
   }

   @Test
   public void testAdd() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test addX, addY, addZ, addS
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple1.setS(sOld);

         tuple1.addX(x);
         assertEquals(tuple1.getX(), xOld + x, getEpsilon());
         tuple1.addY(y);
         assertEquals(tuple1.getY(), yOld + y, getEpsilon());
         tuple1.addZ(z);
         assertEquals(tuple1.getZ(), zOld + z, getEpsilon());
         tuple1.addS(s);
         assertEquals(tuple1.getS(), sOld + s, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(double x, double y)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple1.setS(sOld);

         tuple1.add(x, y, z, s);
         assertEquals(tuple1.getX(), xOld + x, getEpsilon());
         assertEquals(tuple1.getY(), yOld + y, getEpsilon());
         assertEquals(tuple1.getZ(), zOld + z, getEpsilon());
         assertEquals(tuple1.getS(), sOld + s, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(TupleBasics other)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple2.setS(random.nextDouble());
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple1.setS(sOld);

         tuple1.add(tuple2);
         assertEquals(tuple1.getX(), xOld + tuple2.getX(), getEpsilon());
         assertEquals(tuple1.getY(), yOld + tuple2.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), zOld + tuple2.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), sOld + tuple2.getS(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test add(TupleBasics other)
         tuple1.setX(random.nextDouble());
         tuple1.setY(random.nextDouble());
         tuple1.setZ(random.nextDouble());
         tuple1.setS(random.nextDouble());
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple2.setS(random.nextDouble());
         tuple3.setX(random.nextDouble());
         tuple3.setY(random.nextDouble());
         tuple3.setZ(random.nextDouble());
         tuple3.setS(random.nextDouble());

         tuple1.add(tuple2, tuple3);
         assertEquals(tuple1.getX(), tuple2.getX() + tuple3.getX(), getEpsilon());
         assertEquals(tuple1.getY(), tuple2.getY() + tuple3.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), tuple2.getZ() + tuple3.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), tuple2.getS() + tuple3.getS(), getEpsilon());
      }
   }

   @Test
   public void testSub() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test subX, subY, subZ, subS
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple1.setS(sOld);

         tuple1.subX(x);
         assertEquals(tuple1.getX(), xOld - x, getEpsilon());
         tuple1.subY(y);
         assertEquals(tuple1.getY(), yOld - y, getEpsilon());
         tuple1.subZ(z);
         assertEquals(tuple1.getZ(), zOld - z, getEpsilon());
         tuple1.subS(s);
         assertEquals(tuple1.getS(), sOld - s, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(double x, double y)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         tuple1.set(xOld, yOld, zOld, sOld);

         tuple1.sub(x, y, z, s);
         assertEquals(tuple1.getX(), xOld - x, getEpsilon());
         assertEquals(tuple1.getY(), yOld - y, getEpsilon());
         assertEquals(tuple1.getZ(), zOld - z, getEpsilon());
         assertEquals(tuple1.getS(), sOld - s, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(TupleBasics other)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple2.setS(random.nextDouble());
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple1.setS(sOld);

         tuple1.sub(tuple2);
         assertEquals(tuple1.getX(), xOld - tuple2.getX(), getEpsilon());
         assertEquals(tuple1.getY(), yOld - tuple2.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), zOld - tuple2.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), sOld - tuple2.getS(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test sub(TupleBasics tuple1, TupleBasics tuple2)
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.sub(tuple2, tuple3);
         assertEquals(tuple1.getX(), tuple2.getX() - tuple3.getX(), getEpsilon());
         assertEquals(tuple1.getY(), tuple2.getY() - tuple3.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), tuple2.getZ() - tuple3.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), tuple2.getS() - tuple3.getS(), getEpsilon());
      }
   }

   @Test
   public void testScale() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalarX, double scalarY)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         double scale = random.nextDouble();
         tuple1.set(xOld, yOld, zOld, sOld);

         tuple1.scale(scale);
         assertEquals(tuple1.getX(), xOld * scale, getEpsilon());
         assertEquals(tuple1.getY(), yOld * scale, getEpsilon());
         assertEquals(tuple1.getZ(), zOld * scale, getEpsilon());
         assertEquals(tuple1.getS(), sOld * scale, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalarX, double scalarY)
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         double scaleX = random.nextDouble();
         double scaleY = random.nextDouble();
         double scaleZ = random.nextDouble();
         double scaleS = random.nextDouble();
         tuple1.set(xOld, yOld, zOld, sOld);

         tuple1.scale(scaleX, scaleY, scaleZ, scaleS);
         assertEquals(tuple1.getX(), xOld * scaleX, getEpsilon());
         assertEquals(tuple1.getY(), yOld * scaleY, getEpsilon());
         assertEquals(tuple1.getZ(), zOld * scaleZ, getEpsilon());
         assertEquals(tuple1.getS(), sOld * scaleS, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scale(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.setAndScale(scale, tuple2);
         assertEquals(tuple1.getX(), tuple2.getX() * scale, getEpsilon());
         assertEquals(tuple1.getY(), tuple2.getY() * scale, getEpsilon());
         assertEquals(tuple1.getZ(), tuple2.getZ() * scale, getEpsilon());
         assertEquals(tuple1.getS(), tuple2.getS() * scale, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple1.setS(sOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple2.setS(random.nextDouble());

         tuple1.scaleAdd(scale, tuple2);
         assertEquals(tuple1.getX(), xOld * scale + tuple2.getX(), getEpsilon());
         assertEquals(tuple1.getY(), yOld * scale + tuple2.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), zOld * scale + tuple2.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), sOld * scale + tuple2.getS(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, TupleBasics tuple1, TupleBasics tuple2)
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.scaleAdd(scale, tuple2, tuple3);
         assertEquals(tuple1.getX(), tuple2.getX() * scale + tuple3.getX(), getEpsilon());
         assertEquals(tuple1.getY(), tuple2.getY() * scale + tuple3.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), tuple2.getZ() * scale + tuple3.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), tuple2.getS() * scale + tuple3.getS(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleAdd(double scalar, TupleBasics tuple1, TupleBasics tuple2) with tuple2 == this
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(tuple1);

         tuple1.scaleAdd(scale, tuple2, tuple1);
         assertEquals(tuple1.getX(), tuple2.getX() * scale + tuple3.getX(), getEpsilon());
         assertEquals(tuple1.getY(), tuple2.getY() * scale + tuple3.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), tuple2.getZ() * scale + tuple3.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), tuple2.getS() * scale + tuple3.getS(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, TupleBasics other)
         double scale = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple1.setS(sOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple2.setS(random.nextDouble());

         tuple1.scaleSub(scale, tuple2);
         assertEquals(tuple1.getX(), xOld * scale - tuple2.getX(), getEpsilon());
         assertEquals(tuple1.getY(), yOld * scale - tuple2.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), zOld * scale - tuple2.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), sOld * scale - tuple2.getS(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, TupleBasics tuple1, TupleBasics tuple2)
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.scaleSub(scale, tuple2, tuple3);
         assertEquals(tuple1.getX(), tuple2.getX() * scale - tuple3.getX(), getEpsilon());
         assertEquals(tuple1.getY(), tuple2.getY() * scale - tuple3.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), tuple2.getZ() * scale - tuple3.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), tuple2.getS() * scale - tuple3.getS(), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test scaleSub(double scalar, TupleBasics tuple1, TupleBasics tuple2) with tuple2 == this
         double scale = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(tuple1);

         tuple1.scaleSub(scale, tuple2, tuple1);
         assertEquals(tuple1.getX(), tuple2.getX() * scale - tuple3.getX(), getEpsilon());
         assertEquals(tuple1.getY(), tuple2.getY() * scale - tuple3.getY(), getEpsilon());
         assertEquals(tuple1.getZ(), tuple2.getZ() * scale - tuple3.getZ(), getEpsilon());
         assertEquals(tuple1.getS(), tuple2.getS() * scale - tuple3.getS(), getEpsilon());
      }
   }

   @Test
   public void testInterpolate() throws Exception
   {
      Random random = new Random(621541L);
      T tuple1 = createEmptyTuple();
      T tuple2 = createEmptyTuple();
      T tuple3 = createEmptyTuple();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test interpolate(TupleBasics other, double alpha)
         double alpha = random.nextDouble();
         double xOld = random.nextDouble();
         double yOld = random.nextDouble();
         double zOld = random.nextDouble();
         double sOld = random.nextDouble();
         tuple1.setX(xOld);
         tuple1.setY(yOld);
         tuple1.setZ(zOld);
         tuple1.setS(sOld);
         tuple2.setX(random.nextDouble());
         tuple2.setY(random.nextDouble());
         tuple2.setZ(random.nextDouble());
         tuple2.setS(random.nextDouble());

         tuple1.interpolate(tuple2, alpha);
         assertEquals(tuple1.getX(), EuclidCoreTools.interpolate(xOld, tuple2.getX(), alpha), getEpsilon());
         assertEquals(tuple1.getY(), EuclidCoreTools.interpolate(yOld, tuple2.getY(), alpha), getEpsilon());
         assertEquals(tuple1.getZ(), EuclidCoreTools.interpolate(zOld, tuple2.getZ(), alpha), getEpsilon());
         assertEquals(tuple1.getS(), EuclidCoreTools.interpolate(sOld, tuple2.getS(), alpha), getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test interpolate(TupleBasics tuple1, TupleBasics tuple2, double alpha)
         double alpha = random.nextDouble();
         tuple1.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple2.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
         tuple3.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());

         tuple1.interpolate(tuple2, tuple3, alpha);
         assertEquals(tuple1.getX(), EuclidCoreTools.interpolate(tuple2.getX(), tuple3.getX(), alpha), getEpsilon());
         assertEquals(tuple1.getY(), EuclidCoreTools.interpolate(tuple2.getY(), tuple3.getY(), alpha), getEpsilon());
         assertEquals(tuple1.getZ(), EuclidCoreTools.interpolate(tuple2.getZ(), tuple3.getZ(), alpha), getEpsilon());
         assertEquals(tuple1.getS(), EuclidCoreTools.interpolate(tuple2.getS(), tuple3.getS(), alpha), getEpsilon());
      }
   }

   @Test
   public void testApplyTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = EuclidCoreRandomTools.nextAffineTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         transform.transform(expected);
         actual.set(original);
         actual.applyTransform(transform);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, 10.0 * getEpsilon());
      }
   }

   @Test
   public void testApplyInverseTransform() throws Exception
   {
      Random random = new Random(23523L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         RigidBodyTransform transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         QuaternionBasedTransform transform = EuclidCoreRandomTools.nextQuaternionBasedTransform(random);
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, getEpsilon());
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         AffineTransform transform = new AffineTransform(new RotationScaleMatrix(EuclidCoreRandomTools.nextRotationMatrix(random),
                                                                                 EuclidCoreRandomTools.nextDouble(random, 0.01, 10.0)),
                                                         EuclidCoreRandomTools.nextPoint3D(random, 10.0));
         T original = createRandomTuple(random);
         T expected = createEmptyTuple();
         T actual = createEmptyTuple();

         expected.set(original);
         actual.set(original);
         actual.applyTransform(transform);
         actual.applyInverseTransform(transform);
         EuclidCoreTestTools.assertTuple4DEquals(expected, actual, 10.0 * getEpsilon());
      }
   }

   @Test
   public void testGeometricallyEquals() throws Exception
   {
      Vector4DBasics vectorA;
      Vector4DBasics vectorB;
      Random random = new Random(621541L);

      for (int i = 0; i < ITERATIONS; ++i)
      {
         vectorA = EuclidCoreRandomTools.nextVector4D(random);
         vectorB = EuclidCoreRandomTools.nextVector4D(random);

         if (vectorA.epsilonEquals(vectorB, getEpsilon()))
         {
            assertTrue(vectorA.geometricallyEquals(vectorB, Math.sqrt(3) * getEpsilon()));
         }
         else
         {
            Vector4D diff = new Vector4D();
            diff.sub(vectorA, vectorB);
            if (diff.norm() <= getEpsilon())
            {
               assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
            else
            {
               assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
            }
         }

         vectorA = EuclidCoreRandomTools.nextVector4D(random);
         vectorB = new Vector4D(vectorA);

         assertTrue(vectorA.geometricallyEquals(vectorB, 0));

         vectorB.set(vectorA.getX() + 0.9d * getEpsilon(), vectorA.getY(), vectorA.getZ(), vectorA.getS());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX() + 1.1d * getEpsilon(), vectorA.getY(), vectorA.getZ(), vectorA.getS());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 0.9d * getEpsilon(), vectorA.getZ(), vectorA.getS());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY() + 1.1d * getEpsilon(), vectorA.getZ(), vectorA.getS());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY(), vectorA.getZ() + 0.9d * getEpsilon(), vectorA.getS());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY(), vectorA.getZ() + 1.1d * getEpsilon(), vectorA.getS());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY(), vectorA.getZ(), vectorA.getS() + 0.9d * getEpsilon());

         assertTrue(vectorA.geometricallyEquals(vectorB, getEpsilon()));

         vectorB.set(vectorA.getX(), vectorA.getY(), vectorA.getZ(), vectorA.getS() + 1.1d * getEpsilon());

         assertFalse(vectorA.geometricallyEquals(vectorB, getEpsilon()));
      }
   }
}