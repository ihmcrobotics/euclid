package us.ihmc.euclid.referenceFrame.interfaces;

import static org.junit.Assert.*;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.junit.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class FrameVertex3DSupplierTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final double EPSILON = 1.0e-12;
   public static final int ITERATIONS = 1000;

   @Test
   public void testCreatingEmptySupplier()
   {
      FrameVertex3DSupplier expected = FrameVertex3DSupplier.emptyFrameVertex3DSupplier();
      FrameVertex3DSupplier actual;

      actual = FrameVertex3DSupplier.asFrameVertex3DSupplier();
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex3DSupplier.asFrameVertex3DSupplier(new FramePoint3DReadOnly[0], 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex3DSupplier.asFrameVertex3DSupplier(new FramePoint3DReadOnly[0], 0, 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex3DSupplier.asFrameVertex3DSupplier(Collections.emptyList());
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex3DSupplier.asFrameVertex3DSupplier(Collections.emptyList(), 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex3DSupplier.asFrameVertex3DSupplier(Collections.emptyList(), 0, 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
   }

   @Test
   public void testAsFrameVertex3DSupplier() throws Exception
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex3DSupplier(List<? extends FramePoint3DReadOnly> vertices)
         int numberOfVertices = random.nextInt(200);
         List<FramePoint3D> original = new ArrayList<>();
         while (original.size() < numberOfVertices)
            original.add(EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         FrameVertex3DSupplier supplier = FrameVertex3DSupplier.asFrameVertex3DSupplier(original);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());
         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(original.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex3DSupplier(List<? extends FramePoint3DReadOnly> vertices, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         List<FramePoint3D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         FrameVertex3DSupplier supplier = FrameVertex3DSupplier.asFrameVertex3DSupplier(original, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<FramePoint3D> subList = original.subList(0, numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex3DSupplier(List<? extends FramePoint3DReadOnly> vertices, int startIndex, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         int startIndex = random.nextInt(listSize - numberOfVertices);

         List<FramePoint3D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         FrameVertex3DSupplier supplier = FrameVertex3DSupplier.asFrameVertex3DSupplier(original, startIndex, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<FramePoint3D> subList = original.subList(startIndex, startIndex + numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex3DSupplier(FramePoint3DReadOnly... vertices)
         int numberOfVertices = random.nextInt(200);
         List<FramePoint3D> original = new ArrayList<>();
         while (original.size() < numberOfVertices)
            original.add(EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         FrameVertex3DSupplier supplier = FrameVertex3DSupplier.asFrameVertex3DSupplier(original.toArray(new FramePoint3DReadOnly[0]));

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());
         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(original.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex3DSupplier(FramePoint3DReadOnly[] vertices, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         List<FramePoint3D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         FrameVertex3DSupplier supplier = FrameVertex3DSupplier.asFrameVertex3DSupplier(original.toArray(new FramePoint3DReadOnly[0]), numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<FramePoint3D> subList = original.subList(0, numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex3DSupplier(FramePoint3DReadOnly[] vertices, int startIndex, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         int startIndex = random.nextInt(listSize - numberOfVertices);

         List<FramePoint3D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame));

         FrameVertex3DSupplier supplier = FrameVertex3DSupplier.asFrameVertex3DSupplier(original.toArray(new FramePoint3DReadOnly[0]), startIndex,
                                                                                        numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<FramePoint3D> subList = original.subList(startIndex, startIndex + numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(9017);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int sizeA = random.nextInt(100) + 1;
         List<FramePoint3D> listA = IntStream.range(0, sizeA).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame))
                                             .collect(Collectors.toList());
         List<FramePoint3D> listAPrime = listA.stream().map(FramePoint3D::new).collect(Collectors.toList());

         List<FramePoint3D> listSizeA = IntStream.range(0, sizeA).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame))
                                                 .collect(Collectors.toList());

         int sizeB = random.nextInt(100) + 1;
         List<FramePoint3D> listB = IntStream.range(0, sizeB).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame))
                                             .collect(Collectors.toList());

         assertTrue(FrameVertex3DSupplier.asFrameVertex3DSupplier(listA).equals(FrameVertex3DSupplier.asFrameVertex3DSupplier(listA)));
         assertTrue(FrameVertex3DSupplier.asFrameVertex3DSupplier(listA).equals(FrameVertex3DSupplier.asFrameVertex3DSupplier(listAPrime)));
         assertFalse(FrameVertex3DSupplier.asFrameVertex3DSupplier(listA).equals(FrameVertex3DSupplier.asFrameVertex3DSupplier(listSizeA)));
         assertFalse(FrameVertex3DSupplier.asFrameVertex3DSupplier(listA).equals(FrameVertex3DSupplier.asFrameVertex3DSupplier(listB)));

      }
   }

   @Test
   public void testEpsilonquals() throws Exception
   {
      Random random = new Random(9017);

      for (int i = 0; i < ITERATIONS; i++)
      {
         double epsilon = random.nextDouble();
         int sizeA = random.nextInt(100) + 1;
         List<FramePoint3D> listA = IntStream.range(0, sizeA).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame))
                                             .collect(Collectors.toList());
         List<FramePoint3D> listAPrime = listA.stream().map(FramePoint3D::new).collect(Collectors.toList());
         listAPrime.forEach(p -> p.add(nextDouble(random, epsilon), nextDouble(random, epsilon), nextDouble(random, epsilon)));

         List<FramePoint3D> listSizeA = listA.stream().map(FramePoint3D::new).collect(Collectors.toList());
         listSizeA.forEach(p -> p.add((random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0),
                                      (random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0),
                                      (random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0)));

         int sizeB = random.nextInt(100) + 1;
         List<FramePoint3D> listB = IntStream.range(0, sizeB).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame))
                                             .collect(Collectors.toList());

         assertTrue("Iteration: " + i,
                    FrameVertex3DSupplier.asFrameVertex3DSupplier(listA).epsilonEquals(FrameVertex3DSupplier.asFrameVertex3DSupplier(listA), epsilon));
         assertTrue("Iteration: " + i,
                    FrameVertex3DSupplier.asFrameVertex3DSupplier(listA).epsilonEquals(FrameVertex3DSupplier.asFrameVertex3DSupplier(listAPrime), epsilon));
         assertFalse("Iteration: " + i,
                     FrameVertex3DSupplier.asFrameVertex3DSupplier(listA).epsilonEquals(FrameVertex3DSupplier.asFrameVertex3DSupplier(listSizeA), epsilon));
         assertFalse("Iteration: " + i,
                     FrameVertex3DSupplier.asFrameVertex3DSupplier(listA).epsilonEquals(FrameVertex3DSupplier.asFrameVertex3DSupplier(listB), epsilon));
      }
   }
}
