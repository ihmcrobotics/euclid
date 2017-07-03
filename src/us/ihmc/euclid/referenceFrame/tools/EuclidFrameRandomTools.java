package us.ihmc.euclid.referenceFrame.tools;

import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class EuclidFrameRandomTools
{
   /**
    * Generates a reference frame with a random transform to its parent frame.
    * <p>
    * This is usually used for test purposes.
    * </p>
    *
    * @param frameName the name of the new frame.
    * @param random the random generator to use.
    * @param parentFrame the parent frame of the new reference frame.
    * @return the new random reference frame.
    */
   public static ReferenceFrame generateRandomReferenceFrame(String frameName, Random random, ReferenceFrame parentFrame)
   {
      RigidBodyTransform transformFromParent = EuclidCoreRandomTools.generateRandomRigidBodyTransform(random);
      return ReferenceFrame.constructFrameWithUnchangingTransformFromParent(frameName, parentFrame, transformFromParent);
   }

   /**
    * Creates a tree structure of 20 random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    * 
    * @param random the random generator to use.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random)
   {
      return generateRandomReferenceFrameTree("randomFrame", random, ReferenceFrame.getWorldFrame(), 20);
   }

   /**
    * Creates a tree structure of random reference frames starting off
    * {@code ReferenceFrame.getWorldFrame()}.
    * 
    * @param random the random generator to use.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frame and
    *         {@code ReferenceFrame.getWorldFrame()} at the first index.
    */
   public static ReferenceFrame[] generateRandomReferenceFrameTree(Random random, int numberOfReferenceFrames)
   {
      return generateRandomReferenceFrameTree("randomFrame", random, ReferenceFrame.getWorldFrame(), numberOfReferenceFrames);
   }

   /**
    * Creates a tree structure of random reference frames starting off the given {@code rootFrame}.
    * 
    * @param frameNamePrefix prefix to use when creating each random reference frame.
    * @param random the random generator to use.
    * @param rootFrame the base frame from which the tree is to be expanded.
    * @param numberOfReferenceFrames the number of reference frames to be created.
    * @return the array containing the random reference frame and {@code rootFrame} at the first
    *         index.
    */
   public static ReferenceFrame[] generateRandomReferenceFrameTree(String frameNamePrefix, Random random, ReferenceFrame rootFrame, int numberOfReferenceFrames)
   {
      ReferenceFrame[] referenceFrames = new ReferenceFrame[numberOfReferenceFrames + 1];
      referenceFrames[0] = rootFrame;

      for (int i = 0; i < numberOfReferenceFrames; i++)
      {
         int parentFrameIndex = random.nextInt(i);
         ReferenceFrame parentFrame = referenceFrames[parentFrameIndex];
         referenceFrames[i + 1] = generateRandomReferenceFrame(frameNamePrefix + i, random, parentFrame);
      }

      return referenceFrames;
   }
}
