package us.ihmc.euclid.referenceFrame;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class FrameQuaternionTest<F extends FrameQuaternion, T extends QuaternionBasics & GeometryObject> extends FrameQuaternionReadOnlyTest<F>
{
   public static final double EPSILON = 1e-10;

   @Override
   public F createTuple(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      return (F) new FrameQuaternion(referenceFrame, x, y, z, s);
   }

   @Override
   public F createQuaternionUnsafe(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      F ret = (F) new FrameQuaternion(referenceFrame);
      ret.setUnsafe(x, y, z, s);
      return ret;
   }

   public final F createRandomTuple(Random random, ReferenceFrame referenceFrame)
   {
      return createTuple(referenceFrame, random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   @Test
   public void testSetIncludingFrame() throws Exception
   {
      Random random = new Random(2342);

      ReferenceFrame initialFrame = ReferenceFrame.getWorldFrame();

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double z, double s)
         double x = random.nextDouble();
         double y = random.nextDouble();
         double z = random.nextDouble();
         double s = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);
         QuaternionBasics tuple = new Quaternion();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, x, y, z, s);
         tuple.set(x, y, z, s);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertQuaternionEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
         QuaternionReadOnly input = EuclidCoreRandomTools.generateRandomQuaternion(random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);
         QuaternionBasics tuple = new Quaternion();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         frameTuple.setIncludingFrame(newFrame, input);
         tuple.set(input);
         assertEquals(newFrame, frameTuple.getReferenceFrame());
         EuclidCoreTestTools.assertQuaternionEquals(tuple, frameTuple, EPSILON);
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, double[] tupleArray)
         double[] input = new double[random.nextInt(20)];
         for (int j = 0; j < input.length; j++)
            input[j] = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

         QuaternionBasics tuple = new Quaternion();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertQuaternionEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] tupleArray)
         int startIndex = random.nextInt(10);
         double[] input = new double[random.nextInt(20)];
         for (int j = 0; j < input.length; j++)
            input[j] = random.nextDouble();
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

         QuaternionBasics tuple = new Quaternion();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(startIndex, input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, startIndex, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertQuaternionEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, DenseMatrix64F tupleDenseMatrix)
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

         QuaternionBasics tuple = new Quaternion();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertQuaternionEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startRow, DenseMatrix64F tupleDenseMatrix)
         int startRow = random.nextInt(10);
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

         QuaternionBasics tuple = new Quaternion();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(startRow, input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, startRow, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertQuaternionEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }

      for (int i = 0; i < NUMBER_OF_ITERATIONS; i++)
      { // Tests setIncludingFrame(ReferenceFrame referenceFrame, int startRow, int column, DenseMatrix64F tupleDenseMatrix)
         int startRow = random.nextInt(10);
         int column = random.nextInt(10);
         DenseMatrix64F input = RandomMatrices.createRandom(random.nextInt(20), random.nextInt(20), random);
         ReferenceFrame newFrame = EuclidFrameRandomTools.generateRandomReferenceFrame(random);
         F frameTuple = createRandomTuple(random, initialFrame);

         QuaternionBasics tuple = new Quaternion();
         assertEquals(initialFrame, frameTuple.getReferenceFrame());
         Exception expectedException = null;

         try
         {
            tuple.set(startRow, column, input);
         }
         catch (Exception e)
         {
            expectedException = e;
         }
         try
         {
            frameTuple.setIncludingFrame(newFrame, startRow, column, input);
            if (expectedException != null)
               throw new AssertionError("Should have thrown an exception.");

            assertEquals(newFrame, frameTuple.getReferenceFrame());
            EuclidCoreTestTools.assertQuaternionEquals(tuple, frameTuple, EPSILON);
         }
         catch (Exception e)
         {
            if (expectedException == null)
               throw new AssertionError("Should not have thrown an exception.");
            if (!e.getClass().equals(expectedException.getClass()) || !e.getMessage().equals(e.getMessage()))
               throw new AssertionError("Unexpected exception:\nactual: " + e + "\nexpected: " + expectedException);
         }
      }
   }

   @Override
   @Test
   public void testGetters() throws Exception
   {
      Random random = new Random(621541L);
      F tuple = this.createEmptyTuple();

      int i;
      double x;
      double y;
      double z;
      double s;
      for (i = 0; i < 100; ++i)
      {
         x = random.nextDouble();
         y = random.nextDouble();
         z = random.nextDouble();
         s = random.nextDouble();
         tuple.setUnsafe(x, y, z, s);
         Assert.assertEquals(tuple.getX(), x, this.getEpsilon());
         Assert.assertEquals(tuple.getY(), y, this.getEpsilon());
         Assert.assertEquals(tuple.getZ(), z, this.getEpsilon());
         Assert.assertEquals(tuple.getS(), s, this.getEpsilon());
      }

      float fx;
      float fy;
      float fz;
      float fs;
      for (i = 0; i < 100; ++i)
      {
         fx = random.nextFloat();
         fy = random.nextFloat();
         fz = random.nextFloat();
         fs = random.nextFloat();
         tuple.setUnsafe(fx, fy, fz, fs);
         Assert.assertEquals((double) tuple.getX32(), (double) fx, this.getEpsilon());
         Assert.assertEquals((double) tuple.getY32(), (double) fy, this.getEpsilon());
         Assert.assertEquals((double) tuple.getZ32(), (double) fz, this.getEpsilon());
         Assert.assertEquals((double) tuple.getS32(), (double) fs, this.getEpsilon());
      }

      for (i = 0; i < 100; ++i)
      {
         x = random.nextDouble();
         y = random.nextDouble();
         z = random.nextDouble();
         s = random.nextDouble();
         tuple.setUnsafe(x, y, z, s);
         Assert.assertEquals(tuple.getElement(0), x, this.getEpsilon());
         Assert.assertEquals(tuple.getElement(1), y, this.getEpsilon());
         Assert.assertEquals(tuple.getElement(2), z, this.getEpsilon());
         Assert.assertEquals(tuple.getElement(3), s, this.getEpsilon());

         try
         {
            tuple.getElement(-1);
            Assert.fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {

         }
         catch (Exception e2)
         {
            Assert.fail("Should have thrown IndexOutOfBoundsException.");
         }

         try
         {
            tuple.getElement(4);
            Assert.fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException e)
         {

         }
         catch (Exception e2)
         {
            Assert.fail("Should have thrown IndexOutOfBoundsException.");
         }
      }

      for (i = 0; i < 100; ++i)
      {
         fx = random.nextFloat();
         fy = random.nextFloat();
         fz = random.nextFloat();
         fs = random.nextFloat();
         tuple.setUnsafe((double) fx, (double) fy, (double) fz, (double) fs);
         Assert.assertTrue(tuple.getElement32(0) == (double) fx);
         Assert.assertTrue(tuple.getElement32(1) == (double) fy);
         Assert.assertTrue(tuple.getElement32(2) == (double) fz);
         Assert.assertTrue(tuple.getElement32(3) == (double) fs);

         try
         {
            tuple.getElement32(-1);
            Assert.fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException var15)
         {

         }
         catch (Exception var16)
         {
            Assert.fail("Should have thrown IndexOutOfBoundsException.");
         }

         try
         {
            tuple.getElement32(4);
            Assert.fail("Should have thrown IndexOutOfBoundsException.");
         }
         catch (IndexOutOfBoundsException var13)
         {

         }
         catch (Exception var14)
         {
            Assert.fail("Should have thrown IndexOutOfBoundsException.");
         }
      }

      double[] tupleArray;
      for (i = 0; i < 100; ++i)
      {
         tuple = this.createRandomTuple(random);
         tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         tuple.get(tupleArray);
         Assert.assertTrue(tuple.getX() == tupleArray[0]);
         Assert.assertTrue(tuple.getY() == tupleArray[1]);
         Assert.assertTrue(tuple.getZ() == tupleArray[2]);
         Assert.assertTrue(tuple.getS() == tupleArray[3]);
      }

      for (i = 0; i < 100; ++i)
      {
         tuple = this.createRandomTuple(random);
         tupleArray = new double[] {random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(),
               random.nextDouble()};
         tuple.get(2, tupleArray);
         Assert.assertTrue(tuple.getX() == tupleArray[2]);
         Assert.assertTrue(tuple.getY() == tupleArray[3]);
         Assert.assertTrue(tuple.getZ() == tupleArray[4]);
         Assert.assertTrue(tuple.getS() == tupleArray[5]);
      }

      float[] tupleFArray;
      for (i = 0; i < 100; ++i)
      {
         tuple = this.createRandomTuple(random);
         tupleFArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tuple.get(tupleFArray);
         Assert.assertTrue(tuple.getX32() == tupleFArray[0]);
         Assert.assertTrue(tuple.getY32() == tupleFArray[1]);
         Assert.assertTrue(tuple.getZ32() == tupleFArray[2]);
         Assert.assertTrue(tuple.getS32() == tupleFArray[3]);
      }

      for (i = 0; i < 100; ++i)
      {
         tuple = this.createRandomTuple(random);
         tupleFArray = new float[] {random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat(), random.nextFloat()};
         tuple.get(2, tupleFArray);
         Assert.assertTrue(tuple.getX32() == tupleFArray[2]);
         Assert.assertTrue(tuple.getY32() == tupleFArray[3]);
         Assert.assertTrue(tuple.getZ32() == tupleFArray[4]);
         Assert.assertTrue(tuple.getS32() == tupleFArray[5]);
      }

      int index;
      DenseMatrix64F matrix;
      for (i = 0; i < 100; ++i)
      {
         matrix = new DenseMatrix64F(10, 5);

         for (index = 0; index < matrix.getNumElements(); ++index)
         {
            matrix.set(index, random.nextDouble());
         }

         tuple.get(matrix);
         Assert.assertTrue(tuple.getX() == matrix.get(0, 0));
         Assert.assertTrue(tuple.getY() == matrix.get(1, 0));
         Assert.assertTrue(tuple.getZ() == matrix.get(2, 0));
         Assert.assertTrue(tuple.getS() == matrix.get(3, 0));
      }

      for (i = 0; i < 100; ++i)
      {
         matrix = new DenseMatrix64F(10, 5);

         for (index = 0; index < matrix.getNumElements(); ++index)
         {
            matrix.set(index, random.nextDouble());
         }

         tuple.get(2, matrix);
         Assert.assertTrue(tuple.getX() == matrix.get(2, 0));
         Assert.assertTrue(tuple.getY() == matrix.get(3, 0));
         Assert.assertTrue(tuple.getZ() == matrix.get(4, 0));
         Assert.assertTrue(tuple.getS() == matrix.get(5, 0));
      }

      for (i = 0; i < 100; ++i)
      {
         matrix = new DenseMatrix64F(10, 5);

         for (index = 0; index < matrix.getNumElements(); ++index)
         {
            matrix.set(index, random.nextDouble());
         }

         tuple.get(2, 4, matrix);
         Assert.assertTrue(tuple.getX() == matrix.get(2, 4));
         Assert.assertTrue(tuple.getY() == matrix.get(3, 4));
         Assert.assertTrue(tuple.getZ() == matrix.get(4, 4));
         Assert.assertTrue(tuple.getS() == matrix.get(5, 4));
      }
   }

   @Override
   @Test
   public void testLengthSquared() {
      Random random = new Random(312310L);

      for(int i = 0; i < 100; ++i) {
         F tuple1 = this.createRandomTuple(random);
         double length1 = tuple1.norm();
         double scalar = EuclidCoreRandomTools.generateRandomDouble(random, 0.0D, 10.0D);
         F tuple2 = this.createQuaternionUnsafe(scalar * tuple1.getX(), scalar * tuple1.getY(), scalar * tuple1.getZ(), scalar * tuple1.getS());
         double expectedLength2 = scalar * length1;
         double actualLength2 = tuple2.normSquared();
         Assert.assertEquals(expectedLength2, Math.sqrt(actualLength2), 5.0D * this.getEpsilon());
      }
   }

   @Override
   @Test
   public void testLength() {
      Random random = new Random(312310L);

      for(int i = 0; i < 100; ++i) {
         F tuple1 = this.createRandomTuple(random);
         double length1 = tuple1.norm();
         double scalar = EuclidCoreRandomTools.generateRandomDouble(random, 0.0D, 10.0D);
         F tuple2 = this.createQuaternionUnsafe(scalar * tuple1.getX(), scalar * tuple1.getY(), scalar * tuple1.getZ(), scalar * tuple1.getS());
         double expectedLength2 = scalar * length1;
         double actualLength2 = tuple2.norm();
         Assert.assertEquals(expectedLength2, actualLength2, 5.0D * this.getEpsilon());
      }
   }

   @Override
   @Test
   public void testEpsilonEquals() throws Exception {
      Random random = new Random(621541L);
      double epsilon = random.nextDouble();
      F tuple = this.createRandomTuple(random);
      double x = tuple.getX();
      double y = tuple.getY();
      double z = tuple.getZ();
      double s = tuple.getS();
      Assert.assertTrue(tuple.epsilonEquals(this.createQuaternionUnsafe(x + 0.999D * epsilon, y, z, s), epsilon));
      Assert.assertTrue(tuple.epsilonEquals(this.createQuaternionUnsafe(x - 0.999D * epsilon, y, z, s), epsilon));
      Assert.assertTrue(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y + 0.999D * epsilon, z, s), epsilon));
      Assert.assertTrue(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y - 0.999D * epsilon, z, s), epsilon));
      Assert.assertTrue(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y, z + 0.999D * epsilon, s), epsilon));
      Assert.assertTrue(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y, z - 0.999D * epsilon, s), epsilon));
      Assert.assertTrue(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y, z, s + 0.999D * epsilon), epsilon));
      Assert.assertTrue(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y, z, s - 0.999D * epsilon), epsilon));
      Assert.assertFalse(tuple.epsilonEquals(this.createQuaternionUnsafe(x + 1.001D * epsilon, y, z, s), epsilon));
      Assert.assertFalse(tuple.epsilonEquals(this.createQuaternionUnsafe(x - 1.001D * epsilon, y, z, s), epsilon));
      Assert.assertFalse(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y + 1.001D * epsilon, z, s), epsilon));
      Assert.assertFalse(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y - 1.001D * epsilon, z, s), epsilon));
      Assert.assertFalse(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y, z + 1.001D * epsilon, s), epsilon));
      Assert.assertFalse(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y, z - 1.001D * epsilon, s), epsilon));
      Assert.assertFalse(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y, z, s + 1.001D * epsilon), epsilon));
      Assert.assertFalse(tuple.epsilonEquals(this.createQuaternionUnsafe(x, y, z, s - 1.001D * epsilon), epsilon));
   }
}
