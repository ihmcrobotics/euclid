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
import java.util.stream.IntStream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Assertions;
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

   @AfterEach
   public void cleanup()
   {
      ReferenceFrame.getWorldFrame().clearChildren();
   }

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
         ReferenceFrame constantFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("constantA" + i, world, expected);

         EuclidCoreTestTools.assertGeometricallyEquals(expected, constantFrame.getTransformToParent(), EPSILON);
         EuclidCoreTestTools.assertGeometricallyEquals(expected, constantFrame.getTransformToDesiredFrame(world), EPSILON);

         constantFrame.getTransformToParent(actual);
         EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPSILON);
         constantFrame.getTransformToDesiredFrame(actual, world);
         EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test with constructFrameWithUnchangingTransformFromParent
         RigidBodyTransform expected = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform actual = new RigidBodyTransform();
         ReferenceFrame constantFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("constantB" + i, world, expected);
         expected.invert();

         EuclidCoreTestTools.assertGeometricallyEquals(expected, constantFrame.getTransformToParent(), EPSILON);
         EuclidCoreTestTools.assertGeometricallyEquals(expected, constantFrame.getTransformToDesiredFrame(world), EPSILON);

         constantFrame.getTransformToParent(actual);
         EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPSILON);
         constantFrame.getTransformToDesiredFrame(actual, world);
         EuclidCoreTestTools.assertGeometricallyEquals(expected, actual, EPSILON);
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

         EuclidFrameRandomTools.nextReferenceFrameTree("tree" + i + "_", random, worldFrame, 100000);
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

   private static RandomlyChangingFrame[] nextRandomlyChangingFrameTree(String frameNamePrefix,
                                                                        Random random,
                                                                        ReferenceFrame rootFrame,
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
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree("tree" + i + "_", random);

         ReferenceFrame frameA = treeFrame[random.nextInt(treeFrame.length)];
         ReferenceFrame frameB = treeFrame[random.nextInt(treeFrame.length)];

         RigidBodyTransform shouldBeIdentity = new RigidBodyTransform(frameB.getTransformToDesiredFrame(frameA));
         shouldBeIdentity.multiply(frameA.getTransformToDesiredFrame(frameB));

         EuclidCoreTestTools.assertGeometricallyEquals(new RigidBodyTransform(), shouldBeIdentity, EPSILON);
      }
   }

   @Test
   public void testGetTransformToParents()
   {
      Random random = new Random(87);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree("tree" + i + "_", random);

         ReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         checkRepInvariants(frame);

         ReferenceFrame parent = frame.getParent();
         if (parent != null)
         {
            RigidBodyTransform transformToParentOne = frame.getTransformToParent();
            RigidBodyTransform transformToParentTwo = frame.getTransformToDesiredFrame(parent);
            EuclidCoreTestTools.assertGeometricallyEquals(transformToParentOne, transformToParentTwo, EPSILON);

            RigidBodyTransform transformToParentThree = parent.getTransformToDesiredFrame(frame);
            transformToParentThree.invert();
            EuclidCoreTestTools.assertGeometricallyEquals(transformToParentOne, transformToParentThree, EPSILON);
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
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree("treeA" + i + "_", random);

         ReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(worldFrame);
         verifyTransformToRootByClimbingTree(frame, transformToRootOne);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame anotherRoot = ReferenceFrameTools.constructARootFrame("anotherRoot");

         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree("blop" + i + "_", random, anotherRoot, 20);

         ReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(anotherRoot);
         verifyTransformToRootByClimbingTree(frame, transformToRootOne);
      }

      for (int i = 0; i < ITERATIONS; i++)
      {
         RandomlyChangingFrame[] treeFrame = nextRandomlyChangingFrameTree("treeB" + i + "_", random, 100);

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
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree("tree" + i + "_", random);
         ReferenceFrame frame = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformToSelf = frame.getTransformToDesiredFrame(frame);
         EuclidCoreTestTools.assertGeometricallyEquals(new RigidBodyTransform(), transformToSelf, EPSILON);
      }
   }

   @Test
   public void testGetTransformBetweenFrames()
   {
      Random random = new Random(1776L);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] treeFrame = EuclidFrameRandomTools.nextReferenceFrameTree("tree" + i + "_", random);
         ReferenceFrame frame1 = treeFrame[random.nextInt(treeFrame.length)];
         ReferenceFrame frame2 = treeFrame[random.nextInt(treeFrame.length)];

         RigidBodyTransform transformOne = frame1.getTransformToDesiredFrame(frame2);
         RigidBodyTransform transformTwo = frame2.getTransformToDesiredFrame(frame1);

         transformTwo.invert();

         EuclidCoreTestTools.assertGeometricallyEquals(transformOne, transformTwo, EPSILON);

         RigidBodyTransform transformThree = new RigidBodyTransform();
         if (frame2.getTransformToRoot() != null)
            transformThree.setAndInvert(frame2.getTransformToRoot());
         if (frame1.getTransformToRoot() != null)
            transformThree.multiply(frame1.getTransformToRoot());
         EuclidCoreTestTools.assertGeometricallyEquals(transformOne, transformThree, EPSILON);

         RigidBodyTransform transformFour = new RigidBodyTransform();
         transformFour.set(worldFrame.getTransformToDesiredFrame(frame2));
         transformFour.multiply(frame1.getTransformToDesiredFrame(worldFrame));
         EuclidCoreTestTools.assertGeometricallyEquals(transformOne, transformFour, EPSILON);

         ReferenceFrame frame3 = treeFrame[random.nextInt(treeFrame.length)];
         RigidBodyTransform transformFive = new RigidBodyTransform();
         transformFive.set(frame3.getTransformToDesiredFrame(frame2));
         transformFive.multiply(frame1.getTransformToDesiredFrame(frame3));
         EuclidCoreTestTools.assertGeometricallyEquals(transformOne, transformFive, EPSILON);
      }
   }

   private void verifyTransformToRootByClimbingTree(ReferenceFrame frame, RigidBodyTransform transformToRootOne)
   {
      RigidBodyTransform transformToRootTwo = getTransformToDesiredAncestorByClimbingTree(frame, null);
      EuclidCoreTestTools.assertGeometricallyEquals(transformToRootOne, transformToRootTwo, EPSILON);
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

         ReferenceFrame[] referenceFrames = EuclidFrameRandomTools.nextReferenceFrameTree("tree" + i + "_", random);

         ReferenceFrame initialFrame = referenceFrames[random.nextInt(referenceFrames.length)];
         ReferenceFrame desiredFrame = referenceFrames[random.nextInt(referenceFrames.length)];

         RigidBodyTransform transform = initialFrame.getTransformToDesiredFrame(desiredFrame);
         transform.transform(expected);
         initialFrame.transformFromThisToDesiredFrame(desiredFrame, actual);

         EuclidCoreTestTools.assertEquals("Iteration #" + i, expected, actual, EPSILON);

         desiredFrame.transformFromThisToDesiredFrame(initialFrame, actual);
         EuclidCoreTestTools.assertEquals(original, actual, EPSILON);

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
      ReferenceFrameTools.clearWorldFrameTree();
      assertEquals(0L, ReferenceFrameTools.getWorldFrame().getFrameIndex());
      existingIds.add(0L);

      List<Long> frameIndices = new ArrayList<>();

      for (int i = 0; i < ITERATIONS; i++)
      {
         ReferenceFrame[] frameTree = EuclidFrameRandomTools.nextReferenceFrameTree("treeA" + i + "_", random);
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
         ReferenceFrame[] frameTree = EuclidFrameRandomTools.nextReferenceFrameTree("treeB" + i + "_", random);
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

   @Test
   public void testNameRestrictionWhenCreatingFrames()
   {
      String frameName0 = "testName0";
      String frameName1 = "testName1";

      /* Test with NameRestrictionLevel.NONE at various levels */
      worldFrame.clearChildren();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);

      ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      ReferenceFrame frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameA, new RigidBodyTransform());
      ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameB, new RigidBodyTransform());

      /* Test with NameRestrictionLevel.NAME_ID at various levels */
      worldFrame.clearChildren();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NAME_ID);

      frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      try
      {
         frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
         fail("Should have thrown a RuntimeException");
      }
      catch (RuntimeException e)
      {
         // good
      }

      ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameA, new RigidBodyTransform());
      try
      {
         ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameB, new RigidBodyTransform());
         fail("Should have thrown a RuntimeException");
      }
      catch (RuntimeException e)
      {
         // good
      }

      /* Test with NameRestrictionLevel.FRAME_NAME at various levels */
      worldFrame.clearChildren();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.FRAME_NAME);

      frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, worldFrame, new RigidBodyTransform());

      try
      {
         ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameA, new RigidBodyTransform());
         fail("Should have thrown a RuntimeException");
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, frameB, new RigidBodyTransform());
         fail("Should have thrown a RuntimeException");
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testChangingNameRestrictionOnExistingFrames()
   {
      String frameName0 = "testName0";
      String frameName1 = "testName1";
      String frameName2 = "testName2";

      /* Test valid change to NAME_ID */
      worldFrame.clearChildren();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);

      ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      ReferenceFrame frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, worldFrame, new RigidBodyTransform());
      ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameA, new RigidBodyTransform());
      ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, frameB, new RigidBodyTransform());
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NAME_ID);

      /* Test invalid change to NAME_ID */
      worldFrame.clearChildren();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);

      frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameA, new RigidBodyTransform());
      ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameA, new RigidBodyTransform());

      try
      {
         worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NAME_ID);
         fail("Should have thrown a RuntimeException");
      }
      catch (RuntimeException e)
      {
         // good
      }

      /* Test valid change to FRAME_NAME */
      worldFrame.clearChildren();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);

      frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, worldFrame, new RigidBodyTransform());
      ReferenceFrame frameC = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName2, frameB, new RigidBodyTransform());
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.FRAME_NAME);

      /* Test invalid change to FRAME_NAME */
      worldFrame.clearChildren();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);

      frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, worldFrame, new RigidBodyTransform());
      frameC = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName2, frameB, new RigidBodyTransform());
      ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, frameC, new RigidBodyTransform());

      try
      {
         worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.FRAME_NAME);
         fail("Should have thrown a RuntimeException");
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testNameRestrictionWithRemovedFrames()
   {
      String frameName0 = "testName0";
      String frameName1 = "testName1";
      String frameName2 = "testName2";

      /* Try removing frame that would otherwise invalidate changing to NAME_ID */
      worldFrame.clearChildren();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);

      ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      ReferenceFrame frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameA, new RigidBodyTransform());
      ReferenceFrame frameC = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, frameA, new RigidBodyTransform());
      frameC.remove();

      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NAME_ID);

      /* Try removing frame that would otherwise invalidate changing to FRAME_NAME */
      worldFrame.clearChildren();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);

      frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, worldFrame, new RigidBodyTransform());
      frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName1, worldFrame, new RigidBodyTransform());
      frameC = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName2, frameB, new RigidBodyTransform());
      ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent(frameName0, frameC, new RigidBodyTransform());

      frameC.remove();
      worldFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.FRAME_NAME);
   }

   @Test
   public void testNameRestrictionChanges()
   {
      // Root frames without children allow all restriction level changes
      ReferenceFrame rootFrame = ReferenceFrameTools.constructARootFrame("root");
      rootFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.FRAME_NAME);
      rootFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NAME_ID);
      rootFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NONE);

      // Non-root frames or frames with children cannot have less restriction
      rootFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.FRAME_NAME);
      ReferenceFrame childFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("testFrame", rootFrame, new RigidBodyTransform());

      Assertions.assertEquals(childFrame.getNameRestrictionLevel(), FrameNameRestrictionLevel.FRAME_NAME, "Child frame did not inherit the parent's restriction level");
      try
      {
         rootFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NAME_ID);
         fail("Should have thrown a RuntimeException");
      }
      catch (RuntimeException e)
      {
         // good
      }

      try
      {
         childFrame.setNameRestrictionLevel(FrameNameRestrictionLevel.NAME_ID);
         fail("Should have thrown a RuntimeException");
      }
      catch (RuntimeException e)
      {
         // good
      }
   }

   @Test
   public void testAncestorCheck()
   {
      ReferenceFrame rootA = ReferenceFrameTools.constructARootFrame("rootA");
      ReferenceFrame rootB = ReferenceFrameTools.constructARootFrame("rootB");

      ReferenceFrame childA0 = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("childA0", rootA, new RigidBodyTransform());
      ReferenceFrame childA1 = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("childA1", childA0, new RigidBodyTransform());

      ReferenceFrame childB0 = ReferenceFrameTools.constructFrameWithUnchangingTransformFromParent("childB0", rootB, new RigidBodyTransform());

      childA0.verifyIsAncestor(rootA);
      childA1.verifyIsAncestor(rootA);
      childA1.verifyIsAncestor(childA0);

      try
      {
         childB0.verifyIsAncestor(rootA);
         fail("Invalid ancestor check");
      }
      catch (RuntimeException e)
      {
      }

      try
      {
         rootA.verifyIsAncestor(rootB);
         fail("Invalid ancestor check");
      }
      catch (RuntimeException e)
      {
      }

      try
      {
         childA0.verifyIsAncestor(childA0);
         fail("Invalid ancestor check");
      }
      catch (RuntimeException e)
      {
      }

      try
      {
         childA0.verifyIsAncestor(childA1);
         fail("Invalid ancestor check");
      }
      catch (RuntimeException e)
      {
      }
   }

   @Deprecated
   @Test
   public void testDisabling() throws InstantiationException, IllegalAccessException
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

   @Test
   public void testEfficientComputeTransformMultiThreaded()
   {
      Random random = new Random(343);

      int numberOfFrames = 100;
      ReferenceFrame[] frames = EuclidFrameRandomTools.nextReferenceFrameTree(random, numberOfFrames);
      Thread mainThread = Thread.currentThread();
      ReferenceFrame.getWorldFrame().setTreeUpdateCondition(f -> Thread.currentThread() == mainThread);

      // If there's a concurrent modification, the following will throw a NoARotationMatrixException.
      IntStream.range(0, 10000000).parallel().forEach(i ->
      {
         ReferenceFrame.nextTransformToRootID++;
         ReferenceFrame.getWorldFrame().update();
         frames[random.nextInt(numberOfFrames)].getTransformToRoot();
      });
   }

   public static void main(String[] args)
   {
      String targetTests = EuclidTestConstants.class.getName();
      String targetClasses = ReferenceFrame.class.getName() + " " + ReferenceFrameTools.class.getName();
      EuclidMutationTesting.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
