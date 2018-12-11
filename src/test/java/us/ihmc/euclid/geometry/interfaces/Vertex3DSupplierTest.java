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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Vertex3DSupplierTest
{
   public static final double EPSILON = 1.0e-12;
   @Test
   public void testCreatingEmptySupplier()
   {
      Vertex3DSupplier expected = Vertex3DSupplier.emptyVertex3DSupplier();
      Vertex3DSupplier actual;

      actual = Vertex3DSupplier.asVertex3DSupplier();
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex3DSupplier.asVertex3DSupplier(new Point3DReadOnly[0], 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex3DSupplier.asVertex3DSupplier(new Point3DReadOnly[0], 0, 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex3DSupplier.asVertex3DSupplier(Collections.emptyList());
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex3DSupplier.asVertex3DSupplier(Collections.emptyList(), 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
      actual = Vertex3DSupplier.asVertex3DSupplier(Collections.emptyList(), 0, 0);
      assertEquals(expected, actual);
      assertTrue(expected.epsilonEquals(actual, EPSILON));
   }

   @Test
   public void testAsVertex3DSupplier() throws Exception
   {
      Random random = new Random(23423);

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex3DSupplier(List<? extends Point3DReadOnly> vertices)
         int numberOfVertices = random.nextInt(200);
         List<Point3D> original = new ArrayList<>();
         while (original.size() < numberOfVertices)
            original.add(EuclidCoreRandomTools.nextPoint3D(random));

         Vertex3DSupplier supplier = Vertex3DSupplier.asVertex3DSupplier(original);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());
         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(original.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex3DSupplier(List<? extends Point3DReadOnly> vertices, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         List<Point3D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint3D(random));

         Vertex3DSupplier supplier = Vertex3DSupplier.asVertex3DSupplier(original, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point3D> subList = original.subList(0, numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex3DSupplier(List<? extends Point3DReadOnly> vertices, int startIndex, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         int startIndex = random.nextInt(listSize - numberOfVertices);

         List<Point3D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint3D(random));

         Vertex3DSupplier supplier = Vertex3DSupplier.asVertex3DSupplier(original, startIndex, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point3D> subList = original.subList(startIndex, startIndex + numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex3DSupplier(Point3DReadOnly... vertices)
         int numberOfVertices = random.nextInt(200);
         List<Point3D> original = new ArrayList<>();
         while (original.size() < numberOfVertices)
            original.add(EuclidCoreRandomTools.nextPoint3D(random));

         Vertex3DSupplier supplier = Vertex3DSupplier.asVertex3DSupplier(original.toArray(new Point3DReadOnly[0]));

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());
         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(original.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex3DSupplier(Point3DReadOnly[] vertices, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         List<Point3D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint3D(random));

         Vertex3DSupplier supplier = Vertex3DSupplier.asVertex3DSupplier(original.toArray(new Point3DReadOnly[0]), numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point3D> subList = original.subList(0, numberOfVertices);

         for (int j = 0; j < numberOfVertices; j++)
            assertTrue(subList.get(j) == supplier.getVertex(j));
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // asVertex3DSupplier(Point3DReadOnly[] vertices, int startIndex, int numberOfVertices)
         int listSize = random.nextInt(500) + 1;
         int numberOfVertices = random.nextInt(listSize);
         int startIndex = random.nextInt(listSize - numberOfVertices);

         List<Point3D> original = new ArrayList<>();
         while (original.size() < listSize)
            original.add(EuclidCoreRandomTools.nextPoint3D(random));

         Vertex3DSupplier supplier = Vertex3DSupplier.asVertex3DSupplier(original.toArray(new Point3DReadOnly[0]), startIndex, numberOfVertices);

         assertEquals(numberOfVertices, supplier.getNumberOfVertices());

         List<Point3D> subList = original.subList(startIndex, startIndex + numberOfVertices);

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
         List<Point3D> listA = IntStream.range(0, sizeA).mapToObj(v -> EuclidCoreRandomTools.nextPoint3D(random)).collect(Collectors.toList());
         List<Point3D> listAPrime = listA.stream().map(Point3D::new).collect(Collectors.toList());

         List<Point3D> listSizeA = IntStream.range(0, sizeA).mapToObj(v -> EuclidCoreRandomTools.nextPoint3D(random)).collect(Collectors.toList());

         int sizeB = random.nextInt(100) + 1;
         List<Point3D> listB = IntStream.range(0, sizeB).mapToObj(v -> EuclidCoreRandomTools.nextPoint3D(random)).collect(Collectors.toList());

         assertTrue(Vertex3DSupplier.asVertex3DSupplier(listA).equals(Vertex3DSupplier.asVertex3DSupplier(listA)));
         assertTrue(Vertex3DSupplier.asVertex3DSupplier(listA).equals(Vertex3DSupplier.asVertex3DSupplier(listAPrime)));
         assertFalse(Vertex3DSupplier.asVertex3DSupplier(listA).equals(Vertex3DSupplier.asVertex3DSupplier(listSizeA)));
         assertFalse(Vertex3DSupplier.asVertex3DSupplier(listA).equals(Vertex3DSupplier.asVertex3DSupplier(listB)));

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
         List<Point3D> listA = IntStream.range(0, sizeA).mapToObj(v -> EuclidCoreRandomTools.nextPoint3D(random)).collect(Collectors.toList());
         List<Point3D> listAPrime = listA.stream().map(Point3D::new).collect(Collectors.toList());
         listAPrime.forEach(p -> p.add(nextDouble(random, epsilon), nextDouble(random, epsilon), nextDouble(random, epsilon)));

         List<Point3D> listSizeA = listA.stream().map(Point3D::new).collect(Collectors.toList());
         listSizeA.forEach(p -> p.add((random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0),
                                      (random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0),
                                      (random.nextBoolean() ? -1.0 : 1.0) * nextDouble(random, epsilon, epsilon + 1.0)));

         int sizeB = random.nextInt(100) + 1;
         List<Point3D> listB = IntStream.range(0, sizeB).mapToObj(v -> EuclidCoreRandomTools.nextPoint3D(random)).collect(Collectors.toList());

         assertTrue(Vertex3DSupplier.asVertex3DSupplier(listA).epsilonEquals(Vertex3DSupplier.asVertex3DSupplier(listA), epsilon), "Iteration: " + i);
         assertTrue(Vertex3DSupplier.asVertex3DSupplier(listA).epsilonEquals(Vertex3DSupplier.asVertex3DSupplier(listAPrime), epsilon), "Iteration: " + i);
         assertFalse(Vertex3DSupplier.asVertex3DSupplier(listA).epsilonEquals(Vertex3DSupplier.asVertex3DSupplier(listSizeA), epsilon), "Iteration: " + i);
         assertFalse(Vertex3DSupplier.asVertex3DSupplier(listA).epsilonEquals(Vertex3DSupplier.asVertex3DSupplier(listB), epsilon), "Iteration: " + i);
      }
   }
}
