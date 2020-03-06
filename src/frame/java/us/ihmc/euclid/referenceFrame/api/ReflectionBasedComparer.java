package us.ihmc.euclid.referenceFrame.api;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.List;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.tools.EuclidCoreTools;

class ReflectionBasedComparer
{
   private static final String EPSILON_EQUALS = "epsilonEquals";
   private static final Predicate<Method> epsilonEqualsMethodFilter = m ->
   {
      if (!m.getName().equals(EPSILON_EQUALS) || m.getParameterCount() != 2)
         return false;
      return m.getParameterTypes()[1] == double.class || m.getParameterTypes()[1] == float.class;
   };

   static <T> boolean epsilonEquals(Object framelessParameter, Object frameParameter, double epsilon)
   {
      if (framelessParameter == null && frameParameter == null)
         return true;

      if (framelessParameter != null ^ frameParameter != null)
         return false;

      if (framelessParameter instanceof Clearable && frameParameter instanceof Clearable)
      {
         if (((Clearable) framelessParameter).containsNaN() && ((Clearable) frameParameter).containsNaN())
            return true;
      }

      List<Method> epsilonEqualsMethods = Stream.of(frameParameter.getClass().getMethods()).filter(epsilonEqualsMethodFilter).collect(Collectors.toList());

      if (!epsilonEqualsMethods.isEmpty())
      {
         Method epsilonEqualsMethod = epsilonEqualsMethods.stream().filter(m -> m.getParameterTypes()[0] != Object.class) // Filters the method from FrameGeometryObject.
                                                          .filter(m -> m.getParameterTypes()[0].isAssignableFrom(framelessParameter.getClass())).findAny()
                                                          .orElse(null);
         if (epsilonEqualsMethod != null)
         {
            try
            {
               boolean epsilonEqualsResult = (boolean) epsilonEqualsMethod.invoke(frameParameter, framelessParameter, epsilon);
               return epsilonEqualsResult;
            }
            catch (SecurityException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               System.err.println("Something went wrong when invoking the epsilonEquals method for " + frameParameter.getClass().getSimpleName());
               System.err.println("Objects used as parameters: "
                     + MethodSignature.getMethodSimpleName(boolean.class, EPSILON_EQUALS, framelessParameter.getClass(), double.class));
               e.printStackTrace();
               throw new AssertionError(e);
            }
         }
      }

      if (Double.TYPE.isInstance(framelessParameter) || Float.TYPE.isInstance(framelessParameter))
      {
         if (!Double.TYPE.isInstance(frameParameter) && !Float.TYPE.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return EuclidCoreTools.epsilonEquals((double) framelessParameter, (double) frameParameter, epsilon);
      }

      if (Integer.TYPE.isInstance(framelessParameter) || Long.TYPE.isInstance(framelessParameter))
      {
         if (!Integer.TYPE.isInstance(frameParameter) && !Long.TYPE.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return (long) framelessParameter == (long) frameParameter;
      }

      if (Double.class.isInstance(framelessParameter) || Float.class.isInstance(framelessParameter))
      {
         if (!Double.class.isInstance(frameParameter) && !Float.class.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         double framelessDouble = ((Number) framelessParameter).doubleValue();
         double frameDouble = ((Number) frameParameter).doubleValue();
         return Double.compare(framelessDouble, frameDouble) == 0 || EuclidCoreTools.epsilonEquals(framelessDouble, frameDouble, epsilon);
      }

      if (Integer.class.isInstance(framelessParameter) || Long.class.isInstance(framelessParameter))
      {
         if (!Integer.class.isInstance(frameParameter) && !Long.class.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return ((Number) framelessParameter).longValue() == ((Number) frameParameter).longValue();
      }

      if (Boolean.class.isInstance(framelessParameter))
      {
         if (!Boolean.class.isInstance(frameParameter))
            throw new RuntimeException("Reached unexpected state.");

         return (boolean) framelessParameter == (boolean) frameParameter;
      }

      if (framelessParameter instanceof List)
      {
         if (frameParameter instanceof List)
         {
            List<?> framelessList = (List<?>) framelessParameter;
            List<?> frameList = (List<?>) frameParameter;

            if (framelessList.size() != frameList.size())
               return false;

            for (int i = 0; i < framelessList.size(); i++)
            {
               if (!epsilonEquals(framelessList.get(i), frameList.get(i), epsilon))
                  return false;
            }
            return true;
         }
         else
         {
            throw new RuntimeException("Reached unexpected state.");
         }
      }

      if (framelessParameter instanceof DMatrixRMaj && frameParameter instanceof DMatrixRMaj)
      {
         return MatrixFeatures_DDRM.isEquals((DMatrixRMaj) framelessParameter, (DMatrixRMaj) frameParameter, epsilon);
      }

      if (framelessParameter.getClass().isArray() && frameParameter.getClass().isArray())
         return arrayEquals(framelessParameter, frameParameter, epsilon);

      if (framelessParameter instanceof String && frameParameter instanceof String)
         return true;

      if (framelessParameter instanceof Class && frameParameter instanceof Class)
         return true;

      throw new RuntimeException("Did not expect the following types: " + framelessParameter.getClass().getSimpleName() + " & "
            + frameParameter.getClass().getSimpleName());
   }

   private static boolean arrayEquals(Object framelessParameter, Object frameParameter, double epsilon)
   {
      if (framelessParameter.getClass().getComponentType().isPrimitive())
      {
         if (framelessParameter instanceof int[] && frameParameter instanceof int[])
         {
            int[] framelessArray = (int[]) framelessParameter;
            int[] frameArray = (int[]) frameParameter;

            if (framelessArray.length != frameArray.length)
               return false;
            for (int i = 0; i < framelessArray.length; i++)
            {
               if (Float.compare(framelessArray[i], frameArray[i]) != 0 && !EuclidCoreTools.epsilonEquals(framelessArray[i], frameArray[i], epsilon))
                  return false;
            }
            return true;
         }

         if (framelessParameter instanceof float[] && frameParameter instanceof float[])
         {
            float[] framelessArray = (float[]) framelessParameter;
            float[] frameArray = (float[]) frameParameter;

            if (framelessArray.length != frameArray.length)
               return false;
            for (int i = 0; i < framelessArray.length; i++)
            {
               if (Float.compare(framelessArray[i], frameArray[i]) != 0 && !EuclidCoreTools.epsilonEquals(framelessArray[i], frameArray[i], epsilon))
                  return false;
            }
            return true;
         }

         if (framelessParameter instanceof double[] && frameParameter instanceof double[])
         {
            double[] framelessArray = (double[]) framelessParameter;
            double[] frameArray = (double[]) frameParameter;

            if (framelessArray.length != frameArray.length)
               return false;
            for (int i = 0; i < framelessArray.length; i++)
            {
               if (Double.compare(framelessArray[i], frameArray[i]) != 0 && !EuclidCoreTools.epsilonEquals(framelessArray[i], frameArray[i], epsilon))
                  return false;
            }
            return true;
         }

         throw new RuntimeException("Did not expect the following component types: " + framelessParameter.getClass().getComponentType().getSimpleName() + " & "
               + frameParameter.getClass().getComponentType().getSimpleName());
      }
      else
      {
         Object[] framelessArray = (Object[]) framelessParameter;
         Object[] frameArray = (Object[]) frameParameter;
         if (framelessArray.length != frameArray.length)
            return false;
         for (int i = 0; i < framelessArray.length; i++)
         {
            if (!epsilonEquals(framelessArray[i], frameArray[i], epsilon))
               return false;
         }
         return true;
      }
   }
}
