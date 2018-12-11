package us.ihmc.euclid.referenceFrame.interfaces;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;

public class FrameVertex2DSupplierTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   public static final double EPSILON = 1.0e-12;

   @Test
   public void testCreatingEmptySupplier()
   {
      FrameVertex2DSupplier expected = FrameVertex2DSupplier.emptyFrameVertex2DSupplier();
      FrameVertex2DSupplier actual;

      actual = FrameVertex2DSupplier.asFrameVertex2DSupplier();
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex2DSupplier.asFrameVertex2DSupplier(new FramePoint2DReadOnly[0], 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex2DSupplier.asFrameVertex2DSupplier(new FramePoint2DReadOnly[0], 0, 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex2DSupplier.asFrameVertex2DSupplier(Collections.emptyList());
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex2DSupplier.asFrameVertex2DSupplier(Collections.emptyList(), 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = FrameVertex2DSupplier.asFrameVertex2DSupplier(Collections.emptyList(), 0, 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
   }

   @Test
   public void testAsFrameVertex2DSupplier() throws Exception
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex2DSupplier(List<? extends FramePoint2DReadOnly> vertices)
         int numberOfVertices = random.nextInt(200);
         List<FramePoint2D> original = new ArrayList<>();
         while (original.size() < numberOfVertices)
            original.add(EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame));

         FrameVertex2DSupplier supplier = FrameVertex2DSupplier.asFrameVertex2DSupplier(original);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());
         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(original.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex2DSupplier(List<? extends FramePoint2DReadOnly> vertices, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         List<FramePoint2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame));

         FrameVertex2DSupplier supplier = FrameVertex2DSupplier.asFrameVertex2DSupplier(original, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<FramePoint2D> subList = original.subList(0, numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex2DSupplier(List<? extends FramePoint2DReadOnly> vertices, int startIndex, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         int startIndex = random.nextInt(listSize - numberOfVertices);

         List<FramePoint2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame));

         FrameVertex2DSupplier supplier = FrameVertex2DSupplier.asFrameVertex2DSupplier(original, startIndex, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<FramePoint2D> subList = original.subList(startIndex, startIndex + numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex2DSupplier(FramePoint2DReadOnly... vertices)
         int numberOfVertices = random.nextInt(200);
         List<FramePoint2D> original = new ArrayList<>();
         while (original.size() < numberOfVertices)
            original.add(EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame));

         FrameVertex2DSupplier supplier = FrameVertex2DSupplier.asFrameVertex2DSupplier(original.toArray(new FramePoint2DReadOnly[0]));

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());
         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(original.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex2DSupplier(FramePoint2DReadOnly[] vertices, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         List<FramePoint2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame));

         FrameVertex2DSupplier supplier = FrameVertex2DSupplier.asFrameVertex2DSupplier(original.toArray(new FramePoint2DReadOnly[0]), numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<FramePoint2D> subList = original.subList(0, numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asFrameVertex2DSupplier(FramePoint2DReadOnly[] vertices, int startIndex, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         int startIndex = random.nextInt(listSize - numberOfVertices);

         List<FramePoint2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame));

         FrameVertex2DSupplier supplier = FrameVertex2DSupplier.asFrameVertex2DSupplier(original.toArray(new FramePoint2DReadOnly[0]), startIndex,
                                                                                        numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<FramePoint2D> subList = original.subList(startIndex, startIndex + numberOfVertices);

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
         List<FramePoint2D> listA = IntStream.range(0, sizeA).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame))
                                             .collect(Collectors.toList());
         List<FramePoint2D> listAPrime = listA.stream().map(FramePoint2D::new).collect(Collectors.toList());

         List<FramePoint2D> listSizeA = IntStream.range(0, sizeA).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame))
                                                 .collect(Collectors.toList());

         int sizeB = random.nextInt(100) + 1;
         List<FramePoint2D> listB = IntStream.range(0, sizeB).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame))
                                             .collect(Collectors.toList());

         assertTrue(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA).equals(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA)));
         assertTrue(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA).equals(FrameVertex2DSupplier.asFrameVertex2DSupplier(listAPrime)));
         assertFalse(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA).equals(FrameVertex2DSupplier.asFrameVertex2DSupplier(listSizeA)));
         assertFalse(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA).equals(FrameVertex2DSupplier.asFrameVertex2DSupplier(listB)));

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
         List<FramePoint2D> listA = IntStream.range(0, sizeA).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame))
                                             .collect(Collectors.toList());
         List<FramePoint2D> listAPrime = listA.stream().map(FramePoint2D::new).collect(Collectors.toList());
         listAPrime.forEach(p -> p.add(nextDouble(random, epsilon), nextDouble(random, epsilon)));

         List<FramePoint2D> listSizeA = listA.stream().map(FramePoint2D::new).collect(Collectors.toList());
         listSizeA.forEach(p -> p.add((random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0),
                                      (random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0)));

         int sizeB = random.nextInt(100) + 1;
         List<FramePoint2D> listB = IntStream.range(0, sizeB).mapToObj(v -> EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame))
                                             .collect(Collectors.toList());

         assertTrue(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA).epsilonEquals(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA), epsilon),
                    "Iteration: " + i);
         assertTrue(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA).epsilonEquals(FrameVertex2DSupplier.asFrameVertex2DSupplier(listAPrime), epsilon),
                    "Iteration: " + i);
         assertFalse(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA).epsilonEquals(FrameVertex2DSupplier.asFrameVertex2DSupplier(listSizeA), epsilon),
                     "Iteration: " + i);
         assertFalse(FrameVertex2DSupplier.asFrameVertex2DSupplier(listA).epsilonEquals(FrameVertex2DSupplier.asFrameVertex2DSupplier(listB), epsilon),
                     "Iteration: " + i);
      }
   }
}
