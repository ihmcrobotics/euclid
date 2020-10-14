package us.ihmc.euclid.referenceFrame;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.euclid.EuclidTestConstants.ITERATIONS;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidMutationTesting;
import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

public class ReferenceFrameTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double EPSILON = 1.0e-12;

   /**
    * Test for the issue: <a href="https://github.com/ihmcrobotics/euclid/issues/12">Issue 12</a>.
    */
   @Test
   public void testIssue12()
   {
      Random random = new Random(43563);
      ReferenceFrame world = ReferenceFrameTools.getWorldFrame();

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with constructFrameWithUnchangingTransformToParent
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         ReferenceFrame constantFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("constant" + i, world, expected);

         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, constantFrame.getTransformToParent(), EPSILON);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, constantFrame.getTransformToDesiredFrame(world), EPSILON);

         constantFrame.getTransformToParent(actual);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPSILON);
         constantFrame.getTransformToDesiredFrame(actual, world);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with constructFrameWithUnchangingTransformFromParent
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         ReferenceFrame constantFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("constant" + i, world, expected);
         expected.invert();

         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, constantFrame.getTransformToParent(), EPSILON);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, constantFrame.getTransformToDesiredFrame(world), EPSILON);

         constantFrame.getTransformToParent(actual);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPSILON);
         constantFrame.getTransformToDesiredFrame(actual, world);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testChildrenFramesAreGarbageCollected() throws Exception
   {
      boolean verbose = false;
      Random random = new Random(543);
      Runtime runtime = Runtime.getRuntime();
      int numberOfTests = 10;
      double averageUsedMemoryInMB = 0.0;

      runGarbageCollector();

      for (int i = 0; i < numberOfTests; i++)
      {
         runGarbageCollector();
         long usedMemoryStart = runtime.totalMemory() - runtime.freeMemory() >> 20;

         EuclidFrameRandomTools.nextReferenceFrameTree(random, 100000);
         runGarbageCollector();

         long usedMemoryEnd = runtime.totalMemory() - runtime.freeMemory() >> 20;
         long difference = usedMemoryEnd - usedMemoryStart;
         if (verbose)
         {
            System.out.println("(In MB) usedMemoryStart: " + usedMemoryStart + ", usedMemoryEnd: " + usedMemoryEnd + ", used: " + difference);
         }
         averageUsedMemoryInMB += difference;
      }

      averageUsedMemoryInMB /= numberOfTests;
      assertTrue(averageUsedMemoryInMB < 1.0);
   }

   private static void runGarbageCollector()
   {
      System.gc();
      System.runFinalization();
      try
      {
         Thread.sleep(100);
      }
      catch (InterruptedException e)
      {
      }
   }

   private static RandomlyChangingFrame[] nextRandomlyChangingFrameTree(Random random, int numberOfReferenceFrames)
   {
      return nextRandomlyChangingFrameTree("randomFrame", random, numberOfReferenceFrames);
   }

   private static RandomlyChangingFrame[] nextRandomlyChangingFrameTree(String frameNamePrefix, Random random, int numberOfReferenceFrames)
   {
      return nextRandomlyChangingFrameTree(frameNamePrefix, random, worldFrame, numberOfReferenceFrames);
   }

   private static RandomlyChangingFrame[] nextRandomlyChangingFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame,
                                                                        int numberOfReferenceFrames)
   {
      RandomlyChangingFrame[] referenceFrames = new RandomlyChangingFrame[numberOfReferenceFrames];
      ReferenceFrame[] referenceFramesWithRoot = new ReferenceFrame[numberOfReferenceFrames + 1];
      referenceFramesWithRoot[0] = rootFrame;

      for (int i = 0; i < numberOfReferenceFrames; i++)
      {
         int parentFrameIndex = random.nextInt(i + 1);
         ReferenceFrame parentFrame = referenceFramesWithRoot[parentFrameIndex];
         RandomlyChangingFrame randomlyChangingFrame = new RandomlyChangingFrame(frameNamePrefix + i, parentFrame, random);
         referenceFrames[i] = randomlyChangingFrame;
         referenceFramesWithRoot[i + 1] = randomlyChangingFrame;
      }

      return referenceFrames;
   }

   private static class RandomlyChangingFrame extends ReferenceFrame
   {
      private final Random random;
      private final RigidBodyTransform randomTransform = new RigidBodyTransform();

      public RandomlyChangingFrame(String frameName, ReferenceFrame parentFrame, Random random)
      {
         super(frameName, parentFrame);
         this.random = random;
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         randomTransform.set(EuclidCoreRandomTools.nextRigidBodyTransform(random));
         transformToParent.set(randomTransform);
      }
   }

   @Test
   public void testTypicalExample()
   {
      Random random = new Random(87);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         ReferenceFrame frameA = treeFrame[random.nextInt(treeFrame.length)];
         ReferenceFrame frameB = treeFrame[random.nextInt(treeFrame.length)];

         RigidBodyTransform shouldBeIdentity = new RigidBodyTransform(frameB.getTransformToDesiredFrame(frameA));
         shouldBeIdentity.multiply(frameA.getTransformToDesiredFrame(frameB));

         EuclidCoreTestTools.assertRigidBodyTransformEquals(new RigidBodyTransform(), shouldBeIdentity, EPSILON);
      }
   }

   @Test
   public void testGetTransformToParents()
   {
      Random random = new Random(87);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         ReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         checkRepInvariants(frame);

         ReferenceFrame parent = frame.getParent();
         if (parent != null)
         {
            RigidBodyTransform transformToParentOne = frame.getTransformToParent();
            RigidBodyTransform transformToParentTwo = frame.getTransformToDesiredFrame(parent);
            EuclidCoreTestTools.assertRigidBodyTransformEquals(transformToParentOne, transformToParentTwo, EPSILON);

            RigidBodyTransform transformToParentThree = parent.getTransformToDesiredFrame(frame);
            transformToParentThree.invert();
            EuclidCoreTestTools.assertRigidBodyTransformEquals(transformToParentOne, transformToParentThree, EPSILON);
         }
      }
   }

   private void checkRepInvariants(ReferenceFrame frame)
   {
      List<ReferenceFrame> framesStartingWithRootEndingWithThis = Arrays.asList(frame.getFramesStartingWithRootEndingWithThis());
      int branchLength = framesStartingWithRootEndingWithThis.size();
      if (framesStartingWithRootEndingWithThis.get(branchLength - 1) != frame)
      {
         fail("This must be the last frame in the chain.");
      }

      ReferenceFrame parent = frame.getParent();
      if (parent == null)
      {
         if (branchLength != 1)
         {
            fail("If the parentFrame is null, then this must be a root frame, in which there should be only one frame in the chain.");
         }

         try
         {
            frame.getTransformToParent();
            fail("Root frames don't have transformToParent or transformToRoot defined.");
         }
         catch (NullPointerException e)
         {
            // Expected for the root frame.
         }

         if (frame.getTransformToRoot() != null)
         {
            fail("Root frames don't have transformToParent or transformToRoot defined.");
         }

         if (frame.transformToRootID != 0)
         {
            System.err.println("this ReferenceFrame = " + this);

            fail("transformToRootID = " + frame.transformToRootID + ", Root frames must not be updated.");
         }
      }
      else
      {
         if (framesStartingWithRootEndingWithThis.get(branchLength - 2) != parent)
         {
            fail("The parent must be the second to last frame in the chain.");
         }

         long maxIdSoFar = 0;
         RigidBodyTransform computedTransformToRoot = new RigidBodyTransform();
         for (int i = 1; i < branchLength; i++)
         {
            ReferenceFrame frameInTree = framesStartingWithRootEndingWithThis.get(i);
            computedTransformToRoot.multiply(frameInTree.getTransformToParent());

            long id = frameInTree.transformToRootID;
            if (id < maxIdSoFar)
            {
               // Only need to make sure things are consistent down to where the
               break;
            }

            maxIdSoFar = id;

            if (!frameInTree.getTransformToRoot().epsilonEquals(computedTransformToRoot, 1e-5))
            {
               System.err.println("frame.transformToRoot = " + frameInTree.getTransformToRoot() + ", computedTransformToRoot = " + computedTransformToRoot);
               System.err.println("this = " + this + " frame = " + frameInTree);

               fail("transformToRoot is inconsistent!!");
            }
         }
      }
   }

   @Test
   public void testGetTransformToRoots()
   {
      Random random = new Random(453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         ReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(worldFrame);
         verifyTransformToRootByClimbingTree(frame, transformToRootOne);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame anotherRoot = ReferenceFrameTools.constructARootFrame("anotherRoot");

         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree("blop", random, anotherRoot, 20);

         ReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(anotherRoot);
         verifyTransformToRootByClimbingTree(frame, transformToRootOne);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         RandomlyChangingFrame[] treeFrame = nextRandomlyChangingFrameTree(random, 100);

         int numberOfRandomUpdates = random.nextInt(treeFrame.length / 2) + 1;
         for (int j = 0; j < numberOfRandomUpdates; j++)
         {
            treeFrame[random.nextInt(treeFrame.length)].update();
         }

         for (RandomlyChangingFrame frame : treeFrame)
         {
            verifyTransformToRootByClimbingTree(frame, frame.getTransformToRoot());
         }
      }
   }

   @Test
   public void getTransformToSelf()
   {
      Random random = new Random(453);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformToSelf = frame.getTransformToDesiredFrame(frame);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(new RigidBodyTransform(), transformToSelf, EPSILON);
      }
   }

   @Test
   public void testGetTransformBetweenFrames()
   {
      Random random = new Random(1776L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         ReferenceFrame frame1 = treeFrame[random.nextInt(treeFrame.length)];
         ReferenceFrame frame2 = treeFrame[random.nextInt(treeFrame.length)];

         RigidBodyTransform transformOne = frame1.getTransformToDesiredFrame(frame2);
         RigidBodyTransform transformTwo = frame2.getTransformToDesiredFrame(frame1);

         transformTwo.invert();

         EuclidCoreTestTools.assertRigidBodyTransformEquals(transformOne, transformTwo, EPSILON);

         RigidBodyTransform transformThree = new RigidBodyTransform();
         if (frame2.getTransformToRoot() != null)
            transformThree.setAndInvert(frame2.getTransformToRoot());
         if (frame1.getTransformToRoot() != null)
            transformThree.multiply(frame1.getTransformToRoot());
         EuclidCoreTestTools.assertRigidBodyTransformEquals(transformOne, transformThree, EPSILON);

         RigidBodyTransform transformFour = new RigidBodyTransform();
         transformFour.set(worldFrame.getTransformToDesiredFrame(frame2));
         transformFour.multiply(frame1.getTransformToDesiredFrame(worldFrame));
         EuclidCoreTestTools.assertRigidBodyTransformEquals(transformOne, transformFour, EPSILON);

         ReferenceFrame frame3 = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformFive = new RigidBodyTransform();
         transformFive.set(frame3.getTransformToDesiredFrame(frame2));
         transformFive.multiply(frame1.getTransformToDesiredFrame(frame3));
         EuclidCoreTestTools.assertRigidBodyTransformEquals(transformOne, transformFive, EPSILON);
      }
   }

   private void verifyTransformToRootByClimbingTree(ReferenceFrame frame, RigidBodyTransform transformToRootOne)
   {
      RigidBodyTransform transformToRootTwo = getTransformToDesiredAncestorByClimbingTree(frame, null);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(transformToRootOne, transformToRootTwo, EPSILON);
   }

   private RigidBodyTransform getTransformToDesiredAncestorByClimbingTree(ReferenceFrame frame, ReferenceFrame desiredFrame)
   {
      RigidBodyTransform ret = new RigidBodyTransform();

      ReferenceFrame nextFrame = frame;
      while (true)
      {
         ReferenceFrame parent = nextFrame.getParent();
         if (parent == null)
         {
            break;
         }

         RigidBodyTransform transformToParent = nextFrame.getTransformToParent();
         RigidBodyTransform transform = new RigidBodyTransform(transformToParent);
         transform.multiply(ret);

         ret.set(transform);

         nextFrame = parent;
         if (nextFrame == desiredFrame)
         {
            break;
         }
      }

      return ret;
   }

   @Test
   public void testTransformFromHereToDesiredFrame() throws Exception
   {
      Random random = new Random(9825);

      for (int i = 0; i < ITERATIONS; i++)
      {
         Point3D original = EuclidCoreRandomTools.nextPoint3D(random);
         Point3D expected = new Point3D(original);
         Point3D actual = new Point3D(expected);

         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame desiredFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(desiredFrame);
         transform.transform(expected);
         initialFrame.transformFromThisToDesiredFrame(desiredFrame, actual);

         EuclidCoreTestTools.assertTuple3DEquals("Iteration #" + i, expected, actual, EPSILON);

         desiredFrame.transformFromThisToDesiredFrame(initialFrame, actual);
         EuclidCoreTestTools.assertTuple3DEquals(original, actual, EPSILON);

         ReferenceFrame differentRootFrame = ReferenceFrameTools.constructARootFrame("anotherRootFrame");
         try
         {
            initialFrame.transformFromThisToDesiredFrame(differentRootFrame, actual);
            fail("Should have thrown a RuntimeException");
         }
         catch (RuntimeException e)
         {
            // good
         }
      }
   }

   @Test
   public void testUniqueFrameIndex()
   {
      Random random = new Random(84358345L);
      Set<Long> existingIds = new HashSet<>();
      assertEquals(0L, ReferenceFrameTools.getWorldFrame().getFrameIndex());
      existingIds.add(0L);

      List<Long> frameIndices = new ArrayList<>();

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] frameTree = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         for (ReferenceFrame referenceFrame : frameTree)
         {
            if (referenceFrame == ReferenceFrameTools.getWorldFrame())
            {
               continue;
            }

            long frameIndex = referenceFrame.getFrameIndex();
            assertFalse(existingIds.contains(frameIndex), "Already has ID " + frameIndex);
            existingIds.add(frameIndex);
            frameIndices.add(frameIndex);
         }

         if (random.nextBoolean())
         {
            ReferenceFrame frameToClear = frameTree[random.nextInt(frameTree.length)];
            if (frameToClear == ReferenceFrameTools.getWorldFrame())
               continue;
            // Check that explicitly clearing a part of the tree does not affect the indexing.
            frameToClear.clearChildren();
         }
      }

      // Assert that after clearing, the indexing is being reset.
      ReferenceFrameTools.clearWorldFrameTree();

      random = new Random(84358345L);
      assertEquals(0L, ReferenceFrameTools.getWorldFrame().getFrameIndex());

      int position = 0;
      
      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] frameTree = EuclidFrameRandomTools.nextReferenceFrameTree(random);
         for (ReferenceFrame referenceFrame : frameTree)
         {
            if (referenceFrame == ReferenceFrameTools.getWorldFrame())
            {
               continue;
            }
            
            long frameIndex = referenceFrame.getFrameIndex();
            assertEquals(frameIndices.get(position++), frameIndex);
         }
      }
   }

   // TODO Re-enable when unique names are enforce in ReferenceFrame.
   @Disabled
   @Test
   public void testUniqueNaming()
   {
      Random random = new Random(13L);
      ReferenceFrame someFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
      String frameName = someFrame.getName();
      ReferenceFrame parent = someFrame.getParent();

      try
      {
         ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parent, new RigidBodyTransform());
         fail("Should have thrown a RuntimeException");
      }
      catch (RuntimeException e)
      {
         // good
      }

      someFrame.remove();
      someFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parent, new RigidBodyTransform());

      someFrame.remove();
      someFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parent, new RigidBodyTransform());

      ReferenceFrameTools.clearFrameTree(someFrame);
      someFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parent, new RigidBodyTransform());

      ReferenceFrameTools.clearWorldFrameTree();
      ReferenceFrameTools.constructFrameWithUnchangingTransformToParent(frameName, parent, new RigidBodyTransform());
   }

   @Deprecated
   @Test
   public void testDisabeling() throws InstantiationException, IllegalAccessException
   {
      Random random = new Random(314114L);
      ReferenceFrame[] someFrames = EuclidFrameRandomTools.nextReferenceFrameTree(random);
      ReferenceFrame frameToDisable = someFrames[random.nextInt(someFrames.length - 1) + 1];
      ReferenceFrame[] moreChildren = EuclidFrameRandomTools.nextReferenceFrameTree("AdditionalChild", random, frameToDisable, 10);

      frameToDisable.remove();
      checkDisabled(frameToDisable);
      for (ReferenceFrame child : moreChildren)
      {
         checkDisabled(child);
      }
   }

   private static void checkDisabled(ReferenceFrame frame) throws InstantiationException, IllegalAccessException
   {
      Method[] methods = ReferenceFrame.class.getMethods();
      for (Method method : methods)
      {
         if (Modifier.isStatic(method.getModifiers()))
         {
            continue;
         }
         if (method.getDeclaringClass() != ReferenceFrame.class)
         {
            continue;
         }
         int numberOfParameters = method.getParameterTypes().length;
         if (method.getName().equals("remove") && numberOfParameters == 0)
         {
            continue;
         }

         Object[] parameters = new Object[numberOfParameters];
         for (int paramIdx = 0; paramIdx < numberOfParameters; paramIdx++)
         {
            Class<?> parameterClass = method.getParameterTypes()[paramIdx];
            if (parameterClass.isPrimitive())
            {
               // Only works for some primitive types. If we add a public method that takes a boolean for example we will need to update this.
               parameters[paramIdx] = 0;
            }
         }

         try
         {
            method.invoke(frame, parameters);
            fail("Should have thrown a RuntimeException on " + method.getName());
         }
         catch (Exception e)
         {
            if (e.getCause() instanceof RuntimeException)
            {
               continue;
            }
            else
            {
               fail("There was an exception in " + method.getName() + " but expected a RuntimeException.");
            }
         }
      }
   }

   public static void main(String[] args)
   {
      String targetTests = EuclidTestConstants.class.getName();
      String targetClasses = ReferenceFrame.class.getName() + " " + ReferenceFrameTools.class.getName();
      EuclidMutationTesting.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
