package us.ihmc.euclid.geometry.interfaces;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.euclid.tools.EuclidJUnitTools.*;

import static us.ihmc.euclid.testSuite.EuclidTestSuite.*;
import static us.ihmc.euclid.tools.EuclidCoreRandomTools.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class Vertex2DSupplierTest
{
   public static final double EPSILON = 1.0e-12;
   @Test
   public void testCreatingEmptySupplier()
   {
      Vertex2DSupplier expected = Vertex2DSupplier.emptyVertex2DSupplier();
      Vertex2DSupplier actual;

      actual = Vertex2DSupplier.asVertex2DSupplier();
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex2DSupplier.asVertex2DSupplier(new Point2DReadOnly[0], 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex2DSupplier.asVertex2DSupplier(new Point2DReadOnly[0], 0, 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex2DSupplier.asVertex2DSupplier(Collections.emptyList());
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex2DSupplier.asVertex2DSupplier(Collections.emptyList(), 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex2DSupplier.asVertex2DSupplier(Collections.emptyList(), 0, 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
   }

   @Test
   public void testAsVertex2DSupplier() throws Exception
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex2DSupplier(List<? extends Point2DReadOnly> vertices)
         int numberOfVertices = random.nextInt(200);
         List<Point2D> original = new ArrayList<>();
         while (original.size() < numberOfVertices)
            original.add(EuclidCoreRandomTools.nextPoint2D(random));

         Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(original);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());
         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(original.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex2DSupplier(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         List<Point2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint2D(random));

         Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(original, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point2D> subList = original.subList(0, numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex2DSupplier(List<? extends Point2DReadOnly> vertices, int startIndex, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         int startIndex = random.nextInt(listSize - numberOfVertices);

         List<Point2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint2D(random));

         Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(original, startIndex, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point2D> subList = original.subList(startIndex, startIndex + numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex2DSupplier(Point2DReadOnly... vertices)
         int numberOfVertices = random.nextInt(200);
         List<Point2D> original = new ArrayList<>();
         while (original.size() < numberOfVertices)
            original.add(EuclidCoreRandomTools.nextPoint2D(random));

         Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(original.toArray(new Point2DReadOnly[0]));

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());
         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(original.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex2DSupplier(Point2DReadOnly[] vertices, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         List<Point2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint2D(random));

         Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(original.toArray(new Point2DReadOnly[0]), numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point2D> subList = original.subList(0, numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex2DSupplier(Point2DReadOnly[] vertices, int startIndex, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         int startIndex = random.nextInt(listSize - numberOfVertices);

         List<Point2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint2D(random));

         Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(original.toArray(new Point2DReadOnly[0]), startIndex, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point2D> subList = original.subList(startIndex, startIndex + numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex2DSupplier(double[][] vertices)
         int numberOfVertices = random.nextInt(200);
         List<Point2D> original = new ArrayList<>();
         while (original.size() < numberOfVertices)
            original.add(EuclidCoreRandomTools.nextPoint2D(random));

         double[][] array = new double[numberOfVertices][2];
         for (int j = 0; j < numberOfVertices; j++)
            array[j] = new double[] {original.get(j).getX(), original.get(j).getY()};
         Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(array);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());
         for (int j = 0; j < numberOfVertices; j++)
            assertEquals(original.get(j), supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex2DSupplier(double[][] vertices, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         List<Point2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint2D(random));

         double[][] array = new double[listSize][2];
         for (int j = 0; j < listSize; j++)
            array[j] = new double[] {original.get(j).getX(), original.get(j).getY()};
         Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(array, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point2D> subList = original.subList(0, numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertEquals(subList.get(j), supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex2DSupplier(double[][] vertices, int startIndex, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         int startIndex = random.nextInt(listSize - numberOfVertices);

         List<Point2D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint2D(random));

         double[][] array = new double[listSize][2];
         for (int j = 0; j < listSize; j++)
            array[j] = new double[] {original.get(j).getX(), original.get(j).getY()};
         Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(array, startIndex, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point2D> subList = original.subList(startIndex, startIndex + numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertEquals(subList.get(j), supplier.getVertex(j));
      }
   }

   @Test
   public void testEquals() throws Exception
   {
      Random random = new Random(9017);

      for (int i = 0; i < ITERATIONS; i++)
      {
         int sizeA = random.nextInt(100) + 1;
         List<Point2D> listA = IntStream.range(0, sizeA).mapToObj(v -> EuclidCoreRandomTools.nextPoint2D(random)).collect(Collectors.toList());
         List<Point2D> listAPrime = listA.stream().map(Point2D::new).collect(Collectors.toList());

         List<Point2D> listSizeA = IntStream.range(0, sizeA).mapToObj(v -> EuclidCoreRandomTools.nextPoint2D(random)).collect(Collectors.toList());

         int sizeB = random.nextInt(100) + 1;
         List<Point2D> listB = IntStream.range(0, sizeB).mapToObj(v -> EuclidCoreRandomTools.nextPoint2D(random)).collect(Collectors.toList());

         assertTrue(Vertex2DSupplier.asVertex2DSupplier(listA).equals(Vertex2DSupplier.asVertex2DSupplier(listA)));
         assertTrue(Vertex2DSupplier.asVertex2DSupplier(listA).equals(Vertex2DSupplier.asVertex2DSupplier(listAPrime)));
         assertFalse(Vertex2DSupplier.asVertex2DSupplier(listA).equals(Vertex2DSupplier.asVertex2DSupplier(listSizeA)));
         assertFalse(Vertex2DSupplier.asVertex2DSupplier(listA).equals(Vertex2DSupplier.asVertex2DSupplier(listB)));

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
         List<Point2D> listA = IntStream.range(0, sizeA).mapToObj(v -> EuclidCoreRandomTools.nextPoint2D(random)).collect(Collectors.toList());
         List<Point2D> listAPrime = listA.stream().map(Point2D::new).collect(Collectors.toList());
         listAPrime.forEach(p -> p.add(nextDouble(random, epsilon), nextDouble(random, epsilon)));

         List<Point2D> listSizeA = listA.stream().map(Point2D::new).collect(Collectors.toList());
         listSizeA.forEach(p -> p.add((random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0),
                                      (random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0)));

         int sizeB = random.nextInt(100) + 1;
         List<Point2D> listB = IntStream.range(0, sizeB).mapToObj(v -> EuclidCoreRandomTools.nextPoint2D(random)).collect(Collectors.toList());

         assertTrue(Vertex2DSupplier.asVertex2DSupplier(listA).epsilonEquals(Vertex2DSupplier.asVertex2DSupplier(listA), epsilon), "Iteration: " + i);
         assertTrue(Vertex2DSupplier.asVertex2DSupplier(listA).epsilonEquals(Vertex2DSupplier.asVertex2DSupplier(listAPrime), epsilon), "Iteration: " + i);
         assertFalse(Vertex2DSupplier.asVertex2DSupplier(listA).epsilonEquals(Vertex2DSupplier.asVertex2DSupplier(listSizeA), epsilon), "Iteration: " + i);
         assertFalse(Vertex2DSupplier.asVertex2DSupplier(listA).epsilonEquals(Vertex2DSupplier.asVertex2DSupplier(listB), epsilon), "Iteration: " + i);
      }
   }
}
