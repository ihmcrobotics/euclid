package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collection;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.ConvexPolygon2DBasicsTest;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;

public abstract class FrameConvexPolygon2DBasicsTest<F extends FrameConvexPolygon2DBasics> extends FrameConvexPolygon2DReadOnlyTest<F>
{
   public ConvexPolygon2DBasics createRandomFramelessConvexPolygon2D(Random random)
   {
      return EuclidGeometryRandomTools.nextConvexPolygon2D(random, 1.0, 10);
   }

   @Override
   public void testOverloading() throws Exception
   {
      super.testOverloading();
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameTuple2DBasics.class, Tuple2DBasics.class, true, 1);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFrameConvexPolygon2D,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithConvexPolygon2D() throws Exception
   {
      FrameTypeCopier frameTypeBuilder = (frame, polygon) -> createFrameConvexPolygon2D(frame, (ConvexPolygon2DReadOnly) polygon);
      RandomFramelessTypeBuilder framelessTypeBuilber = this::createRandomFramelessConvexPolygon2D;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("toString")
            && Arrays.stream(m.getParameterTypes()).noneMatch(p -> Collection.class.isAssignableFrom(p));
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                  framelessTypeBuilber,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testConvexPolygon2DBasicsFeatures() throws Exception
   {
      ConvexPolygon2DBasicsTest<FrameConvexPolygon2D> convexPolygonTest = new ConvexPolygon2DBasicsTest<FrameConvexPolygon2D>()
      {
         @Override
         public FrameConvexPolygon2D createEmptyConvexPolygon2D()
         {
            return new FrameConvexPolygon2D();
         }

         @Override
         public FrameConvexPolygon2D createRandomConvexPolygon2D(Random random)
         {
            return EuclidFrameRandomTools.nextFrameConvexPolygon2D(random, ReferenceFrame.getWorldFrame(), 1.0, 50);
         }

         @Override
         public FrameConvexPolygon2D createConvexPolygon2D(Vertex2DSupplier supplier)
         {
            return new FrameConvexPolygon2D(ReferenceFrame.getWorldFrame(), supplier);
         }
      };

      for (Method testMethod : convexPolygonTest.getClass().getMethods())
      {
         if (!testMethod.getName().startsWith("test"))
            continue;
         if (!Modifier.isPublic(testMethod.getModifiers()))
            continue;
         if (Modifier.isStatic(testMethod.getModifiers()))
            continue;

         testMethod.invoke(convexPolygonTest);
      }
   }
}
