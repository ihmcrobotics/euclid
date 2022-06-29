package us.ihmc.euclid.referenceFrame;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.MethodSignature;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;

public abstract class FramePose3DReadOnlyTest<T extends FramePose3DReadOnly>
{
   public abstract T createFramePose(ReferenceFrame referenceFrame, Pose3DReadOnly pose);

   public final T createEmptyFramePose(ReferenceFrame referenceFrame)
   {
      return createFramePose(referenceFrame, new Pose3D());
   }

   public final T createRandomPose(Random random)
   {
      return createRandomFramePose(random, ReferenceFrame.getWorldFrame());
   }

   public final T createRandomFramePose(Random random, ReferenceFrame referenceFrame)
   {
      return createFramePose(referenceFrame, EuclidGeometryRandomTools.nextPose3D(random));
   }

   public final T createRandom2DFramePose(Random random, ReferenceFrame referenceFrame)
   {
      Pose3D pose = new Pose3D();
      pose.setOrientationYawPitchRoll(EuclidCoreRandomTools.nextDouble(random, Math.PI), 0, 0);
      pose.setPosition(EuclidCoreRandomTools.nextPoint2D(random));
      return createFramePose(referenceFrame, pose);
   }

   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(this::createRandomFramePose, methodFilter, EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testOverloading() throws Exception
   {
      List<MethodSignature> signaturesToIgnore = new ArrayList<>();
      for (Method method : Transform.class.getDeclaredMethods())
         signaturesToIgnore.add(new MethodSignature(method.getName(), method.getParameterTypes()));
      for (Method method : RigidBodyTransformReadOnly.class.getDeclaredMethods())
         signaturesToIgnore.add(new MethodSignature(method.getName(), method.getParameterTypes()));
      for (Method method : RigidBodyTransformBasics.class.getDeclaredMethods())
         signaturesToIgnore.add(new MethodSignature(method.getName(), method.getParameterTypes()));
      Predicate<Method> methodFilter = EuclidFrameAPITester.methodFilterFromSignature(signaturesToIgnore);

      EuclidFrameAPITester tester = new EuclidFrameAPITester(new EuclidFrameAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FramePose3DReadOnly.class, Pose3DReadOnly.class, true, 1, methodFilter);
   }
}
