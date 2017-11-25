package us.ihmc.euclid.referenceFrame;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class ReferenceFrameTest
{
   private static final boolean VERBOSE = false;

   private Random random;
   private ReferenceFrame root, frame1, frame2, frame3, frame4, frame5, frame6, frame7, frame8;
   private ReferenceFrame root2, frame9, frame10, frame11;
   private ReferenceFrame[] frames1, frames2;

   private ArrayList<ReferenceFrame> allFramesTogether;

   private LinkedHashMap<String, RigidBodyTransform> transformsForVerification;

   public void setUp()
   {
      random = new Random(23423L);
      transformsForVerification = new LinkedHashMap<>();

      // The structure we'll test is as follows:
      // root                                                 root2
      // frame1                frame2                                    frame9
      // frame3   frame4        frame6  frame7                            frame10 frame11
      // frame5                frame8

      root = ReferenceFrame.constructARootFrame("root");

      // Some are randomly changing and some are random but unchanging:
      frame1 = constructRandomUnchangingFrame("frame1", root);
      frame2 = new RandomlyChangingFrame("frame2", root);
      frame3 = constructRandomUnchangingFrame("frame3", root);
      frame4 = new RandomlyChangingFrame("frame4", frame1);
      frame5 = constructRandomUnchangingFrame("frame5", root);
      frame6 = new RandomlyChangingFrame("frame6", frame2);
      frame7 = constructRandomUnchangingFrame("frame7", root);
      frame8 = new RandomlyChangingFrame("frame8", frame7);

      root2 = ReferenceFrame.constructARootFrame("root2");
      frame9 = constructRandomUnchangingFrame("frame9", root2);
      frame10 = new RandomlyChangingFrame("frame10", frame9);
      frame11 = constructRandomUnchangingFrame("frame11", frame9);

      frames1 = new ReferenceFrame[] {root, frame1, frame2, frame3, frame4, frame5, frame6, frame7, frame8};
      frames2 = new ReferenceFrame[] {root2, frame9, frame10, frame11};

      allFramesTogether = new ArrayList<>();
      addAllFrames(allFramesTogether, frames1);
      addAllFrames(allFramesTogether, frames2);
   }

   private void addAllFrames(ArrayList<ReferenceFrame> arrayList, ReferenceFrame[] frames)
   {
      for (ReferenceFrame frame : frames)
      {
         arrayList.add(frame);
      }
   }

   public void tearDown()
   {
      random = null;
      transformsForVerification = null;
      frames1 = frames2 = null;
      root = frame1 = frame2 = frame3 = frame4 = frame5 = frame6 = frame7 = frame8 = null;
      root2 = frame9 = frame10 = frame11 = null;
      allFramesTogether = null;
   }

   private ReferenceFrame constructRandomUnchangingFrame(String nameOfFrame, ReferenceFrame parentOfFrame)
   {
      RigidBodyTransform randomTransformToParent = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      transformsForVerification.put(nameOfFrame, new RigidBodyTransform(randomTransformToParent));
      ReferenceFrame ret = ReferenceFrame.constructFrameWithUnchangingTransformToParent(nameOfFrame, parentOfFrame, randomTransformToParent);

      return ret;

   }

   private class RandomlyChangingFrame extends ReferenceFrame
   {
      public RandomlyChangingFrame(String frameName, ReferenceFrame parentFrame)
      {
         super(frameName, parentFrame);
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         RigidBodyTransform randomTransform = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
         transformToParent.set(randomTransform);

         transformsForVerification.put(getName(), new RigidBodyTransform(randomTransform));
      }
   }

   @Test
   public void testTypicalExample()
   {
      setUp();

      RigidBodyTransform transformFrom9To10 = frame9.getTransformToDesiredFrame(frame10);
      RigidBodyTransform transformFrom10To9 = frame10.getTransformToDesiredFrame(frame9);

      RigidBodyTransform shouldBeIdentity = new RigidBodyTransform(transformFrom10To9);
      shouldBeIdentity.multiply(transformFrom9To10);

      assertEquals(1.0, shouldBeIdentity.determinantRotationPart(), 1e-7);

      tearDown();
   }

   @Test
   public void testGetTransformToParents()
   {
      setUp();
      updateAllFrames();

      for (ReferenceFrame frame : allFramesTogether)
      {
         frame.checkRepInvariants();

         ReferenceFrame parent = frame.getParent();
         if (parent != null)
         {
            RigidBodyTransform transformToParentOne = frame.getTransformToParent();
            RigidBodyTransform transformToParentTwo = frame.getTransformToDesiredFrame(parent);
            RigidBodyTransform transformToParentThree = transformsForVerification.get(frame.getName());

            verifyTransformsAreEpsilonEqual(transformToParentOne, transformToParentTwo, transformToParentThree);
         }
      }
      tearDown();
   }

   @Test
   public void testGetTransformToRoots()
   {
      setUp();
      updateAllFrames();

      for (ReferenceFrame frame : frames1)
      {
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(root);
         verifyTransformToRootByClimbingTree(frame, transformToRootOne);
      }

      for (ReferenceFrame frame : frames2)
      {
         RigidBodyTransform transformToRootOne = frame.getTransformToDesiredFrame(root2);
         verifyTransformToRootByClimbingTree(frame, transformToRootOne);
      }
      tearDown();
   }

   @Test
   public void getTransformToSelf()
   {
      setUp();
      updateAllFrames();

      for (ReferenceFrame frame : allFramesTogether)
      {
         RigidBodyTransform transformToSelf = frame.getTransformToDesiredFrame(frame);
         verifyTransformsAreEpsilonEqual(transformToSelf, new RigidBodyTransform());
      }
      tearDown();
   }

   @Test
   public void testGetTransformBetweenFrames()
   {
      setUp();
      Random random = new Random(1776L);
      updateAllFrames();

      int numberOfTests = 100000;
      int numberTestsComplete = 0;

      for (int i = 0; i < numberOfTests; i++)
      {
         ReferenceFrame frame1 = selectARandomFrame(random);
         ReferenceFrame frame2 = selectARandomFrame(random);

         updateARandomFrame(random);

         if (frame1.getRootFrame() != frame2.getRootFrame())
         {
            continue;
         }

         RigidBodyTransform transformOne = frame1.getTransformToDesiredFrame(frame2);
         RigidBodyTransform transformTwo = frame2.getTransformToDesiredFrame(frame1);

         transformTwo.invert();

         verifyTransformsAreEpsilonEqual(transformOne, transformTwo);
         numberTestsComplete++;
      }

      if (VERBOSE)
      {
         System.out.println("numberTestsComplete = " + numberTestsComplete);
         System.out.println("nextTransformToRootID = " + ReferenceFrame.nextTransformToRootID);
      }
      tearDown();
   }

   @Test
   public void testGetTransformBetweenFramesTwo()
   {
      setUp();
      Random random = new Random(1776L);
      updateAllFrames();

      int numberOfTests = 100000;
      int numberTestsComplete = 0;

      for (int i = 0; i < numberOfTests; i++)
      {
         ReferenceFrame frame1 = selectARandomFrame(random);
         ReferenceFrame frame2 = selectARandomFrame(random);

         updateARandomFrame(random);

         if (frame1.getRootFrame() != frame2.getRootFrame())
         {
            continue;
         }

         RigidBodyTransform transformOne = frame1.getTransformToDesiredFrame(frame2);
         RigidBodyTransform transformTwo = getTransformToDesiredFrameThroughVerificationTransforms(frame1, frame2);

         verifyTransformsAreEpsilonEqual(transformOne, transformTwo);
         numberTestsComplete++;
      }

      if (VERBOSE)
      {
         System.out.println("numberTestsComplete = " + numberTestsComplete);
         System.out.println("nextTransformToRootID = " + ReferenceFrame.nextTransformToRootID);
      }
      tearDown();
   }

   private void updateARandomFrame(Random random)
   {
      ReferenceFrame frame = selectARandomFrame(random);
      frame.update();
   }

   private ReferenceFrame selectARandomFrame(Random random)
   {
      int index = random.nextInt(allFramesTogether.size());

      return allFramesTogether.get(index);
   }

   private void verifyTransformToRootByClimbingTree(ReferenceFrame frame, RigidBodyTransform transformToRootOne)
   {
      RigidBodyTransform transformToRootTwo = getTransformToDesiredAncestorByClimbingTree(frame, null);
      RigidBodyTransform transformToRootThree = getTransformToDesiredAncestorThroughVerificationTransforms(frame, null);
      verifyTransformsAreEpsilonEqual(transformToRootOne, transformToRootTwo, transformToRootThree);
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

   private RigidBodyTransform getTransformToDesiredAncestorThroughVerificationTransforms(ReferenceFrame frame, ReferenceFrame desiredFrame)
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

         RigidBodyTransform transformToParent = new RigidBodyTransform(transformsForVerification.get(nextFrame.getName()));
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

   private RigidBodyTransform getTransformToDesiredFrameThroughVerificationTransforms(ReferenceFrame frame, ReferenceFrame desiredFrame)
   {
      RigidBodyTransform transformOne = getTransformToDesiredAncestorThroughVerificationTransforms(frame, null);
      RigidBodyTransform transformTwo = getTransformToDesiredAncestorThroughVerificationTransforms(desiredFrame, null);
      transformTwo.invert();
      transformTwo.multiply(transformOne);

      return transformTwo;
   }

   private void verifyTransformsAreEpsilonEqual(RigidBodyTransform transformOne, RigidBodyTransform transformTwo)
   {
      if (!epsilonEquals(transformOne, transformTwo, 1e-2))
      {
         System.err.println("transformOne = " + transformOne);
         System.err.println("transformTwo = " + transformTwo);
         fail();
      }
   }

   private void verifyTransformsAreEpsilonEqual(RigidBodyTransform transformToParentOne, RigidBodyTransform transformToParentTwo,
                                                RigidBodyTransform transformToParentThree)
   {
      if (!epsilonEquals(transformToParentOne, transformToParentTwo, 0.001))
      {
         fail();
      }
      if (!epsilonEquals(transformToParentTwo, transformToParentThree, 0.001))
      {
         fail();
      }
   }

   private boolean epsilonEquals(RigidBodyTransform transformOne, RigidBodyTransform transformTwo, double epsilonPercent)
   {
      double maxDeltaPercent = getMaxDeltaPercent(transformOne, transformTwo);

      return maxDeltaPercent < epsilonPercent;
   }

   private double getMaxDeltaPercent(RigidBodyTransform t1, RigidBodyTransform t2)
   {
      double[] arg1 = new double[16];
      double[] arg2 = new double[16];

      t1.get(arg1);
      t2.get(arg2);

      return getMaxDeltaPercent(arg1, arg2);
   }

   private double getMaxDeltaPercent(double[] arg1, double[] arg2)
   {
      double max = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < arg1.length; i++)
      {
         double absolute1 = Math.abs(arg1[i]);
         double absolute2 = Math.abs(arg2[i]);

         if (absolute1 < 1e-7 && absolute2 < 1e-7)
         {
            if (max < 0.0)
            {
               max = 0.0;
            }
         }

         else
         {
            double absoluteDifference = Math.abs(arg1[i] - arg2[i]);
            double largestOne = Math.max(Math.abs(arg1[i]), Math.abs(arg2[i]));
            double percentDifference = absoluteDifference / largestOne;

            if (percentDifference > max)
            {
               max = percentDifference;
            }
         }
      }

      return max;
   }

   private void updateAllFrames()
   {
      for (ReferenceFrame frame : allFramesTogether)
      {
         frame.update();
      }
   }

}
