package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.Test;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameAPITestTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public abstract class FrameLine3DReadOnlyTest<T extends FrameLine3DReadOnly>
{
   public abstract T createFrameLine(ReferenceFrame referenceFrame, Line3DReadOnly line);

   public final T createRandomLine(Random random)
   {
      return createRandomFrameLine(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFrameLine(Random random, ReferenceFrame referenceFrame)
   {
      return createFrameLine(referenceFrame, EuclidGeometryRandomTools.nextLine3D(random));
   }

   @Test
   public void testOverloading() throws Exception
   {
      Map<String, Class<?>[]> framelessMethodsToIgnore = new HashMap<>();
      framelessMethodsToIgnore.put("get", new Class<?>[] {Point3DBasics.class, Vector3DBasics.class});
      EuclidFrameAPITestTools.assertOverloadingWithFrameObjects(FrameLine3DReadOnly.class, Line3DReadOnly.class, true, 1, framelessMethodsToIgnore);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Random random = new Random(234);
      Predicate<Method> methodFilter = m -> !m.getName().equals("setIncludingFrame") && !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITestTools.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(frame -> createRandomFrameLine(random, frame), false, true, methodFilter);
   }
}
