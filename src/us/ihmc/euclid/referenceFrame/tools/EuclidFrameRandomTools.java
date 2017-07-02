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
}
