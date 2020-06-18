package us.ihmc.euclid.referenceFrame.api;

import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

/**
 * The class provides tools for generating random objects and cloning them.
 * <p>
 * Random generators are separated into 2 categories: frame and frameless type builders. Frame types
 * represent geometries that defined with a {@link ReferenceFrame}, for instance
 * {@link FramePoint3D} or {@link FrameConvexPolygon2D}, while frameless types represent all other
 * types.
 * </p>
 * <p>
 * This interface is part of the API testing framework.
 * </p>
 *
 * @see EuclidFrameAPITester
 * @author Sylvain Bertrand
 */
public class ReflectionBasedBuilder
{
   private final Map<Class<?>, RandomFrameTypeBuilder> frameTypeBuilders = new HashMap<>();
   private final Map<Class<?>, RandomFramelessTypeBuilder> framelessTypeBuilders = new HashMap<>();

   /**
    * Create a new builder.
    * <p>
    * Custom random generators can be registered.
    * </p>
    */
   public ReflectionBasedBuilder()
   {
      framelessTypeBuilders.put(double.class, random -> EuclidCoreRandomTools.nextDouble(random, 10.0));
      framelessTypeBuilders.put(float.class, random -> (float) EuclidCoreRandomTools.nextDouble(random, 10.0));
      framelessTypeBuilders.put(boolean.class, random -> random.nextBoolean());
      framelessTypeBuilders.put(int.class, random -> random.nextInt(1000) - 500);
      framelessTypeBuilders.put(char.class, random -> (char) (random.nextInt(1000) - 500));
      framelessTypeBuilders.put(long.class, random -> (long) (random.nextInt(1000) - 500));

      framelessTypeBuilders.put(double[].class, random -> random.doubles(20, -10.0, 10.0).toArray());
      framelessTypeBuilders.put(float[].class, random -> nextFloatArray(random));
      framelessTypeBuilders.put(boolean[].class, random -> nextBooleanArray(random));
      framelessTypeBuilders.put(int[].class, random -> random.ints(20, -100, 100).toArray());
      framelessTypeBuilders.put(char[].class, random -> nextCharArray(random));
      framelessTypeBuilders.put(long[].class, random -> random.longs(20, -100, 100).toArray());

      framelessTypeBuilders.put(DMatrix.class, random -> EuclidCoreRandomTools.nextDMatrixRMaj(random, 20, 20));
   }

   /**
    * Registers a new random generator that can generate random frame object to this builder.
    *
    * @param frameTypeBuilder the generator to register.
    */
   public void registerFrameTypeBuilder(RandomFrameTypeBuilder frameTypeBuilder)
   {
      frameTypeBuilders.put(frameTypeBuilder.newInstance(new Random(), ReferenceFrame.getWorldFrame()).getClass(), frameTypeBuilder);
   }

   /**
    * Registers a new random generator that can generate random frameless object to this builder.
    *
    * @param framelessTypeBuilder the generator to register.
    */
   public void registerFramelessTypeBuilder(RandomFramelessTypeBuilder framelessTypeBuilder)
   {
      framelessTypeBuilders.put(framelessTypeBuilder.newInstance(new Random()).getClass(), framelessTypeBuilder);
   }

   /**
    * Registers random generators declared as static method in the given {@code classes}.
    * <p>
    * Convention used to extract the random generators:
    * <ul>
    * <li>Any static method declaring a non-void return type and with a single parameter type that is
    * {@link Random} is extracted into a random frameless type generator.
    * <li>Any static method declaring a non-void return type and with two parameter types that are in
    * order {@link Random} and {@link ReferenceFrame} is extracted into a random frame type generator.
    * </ul>
    * </p>
    *
    * @param classes the classes from which to extract random generators.
    */
   public void registerRandomGeneratorClasses(Class<?>... classes)
   {
      for (Class<?> clazz : classes)
         registerRandomGeneratorsFromClass(clazz);
   }

   private void registerRandomGeneratorsFromClass(Class<?> classDeclaringRandomGenerators)
   {
      List<Method> frameTypeRandomGenerators = Stream.of(classDeclaringRandomGenerators.getMethods()).filter(method -> isFrameRandomGenerator(method))
                                                     .collect(Collectors.toList());

      for (Method generator : frameTypeRandomGenerators)
      {
         if (generator.getReturnType() == null || generator.getReturnType() == void.class)
            continue;

         frameTypeBuilders.put(generator.getReturnType(), (random, frame) ->
         {
            try
            {
               return (ReferenceFrameHolder) generator.invoke(null, random, frame);
            }
            catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               throw new RuntimeException(e);
            }
         });
      }

      List<Method> framelessTypeGenerators = Stream.of(classDeclaringRandomGenerators.getMethods()).filter(method -> isFramelessRandomGenerator(method))
                                                   .collect(Collectors.toList());

      for (Method generator : framelessTypeGenerators)
      {
         framelessTypeBuilders.put(generator.getReturnType(), random ->
         {
            try
            {
               return generator.invoke(null, random);
            }
            catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               throw new RuntimeException(e);
            }
         });
      }
   }

   private boolean isFramelessRandomGenerator(Method method)
   {
      return method.getParameterCount() == 1 && method.getParameterTypes()[0] == Random.class;
   }

   private boolean isFrameRandomGenerator(Method method)
   {
      if (method.getParameterCount() != 2)
         return false;
      if (method.getParameterTypes()[0] != Random.class || method.getParameterTypes()[1] != ReferenceFrame.class)
         return false;
      return ReferenceFrameHolder.class.isAssignableFrom(method.getReturnType());
   }

   Object next(Random random, ReferenceFrame frame, Class<?> type)
   {
      if (Object.class.equals(type))
         return null;
      if (Collection.class.equals(type))
         return null;

      Object object = nextFrameType(random, type, frame);
      if (object != null)
         return object;
      object = nextFramelessType(random, type);
      if (object != null)
         return object;
      throw new IllegalStateException("Unknown class: " + type.getSimpleName()
            + "\nThis class can be handled simply by registering a class that declares random generator as method with the following signature:\n\t"
            + type.getSimpleName() + " generatorNameDoesNotMatter(Random)\nor:\n\t" + type.getSimpleName()
            + " generatorNameDoesNotMatter(Random, ReferenceFrame) if the type is a frame type.\nNote that the return type can be any sub-class of "
            + type.getSimpleName() + "\nThe class declaring the generator is to be registered using " + ReflectionBasedBuilder.class.getSimpleName()
            + ".registerRandomGeneratorClasses(Class<?>...).");
   }

   Object[] next(Random random, ReferenceFrame frame, Class<?>[] types)
   {
      Object[] instances = new Object[types.length];

      for (int i = 0; i < types.length; i++)
      {
         Class<?> type = types[i];

         if (type.isArray() && !type.getComponentType().isPrimitive())
         {
            Class<?> componentType = type.getComponentType();

            Object[] array = (Object[]) Array.newInstance(componentType, random.nextInt(15));

            for (int j = 0; j < array.length; j++)
            {
               array[j] = next(random, frame, componentType);
               if (array[j] == null)
                  return null;
            }
            instances[i] = array;
         }
         else
         {
            instances[i] = next(random, frame, type);
            if (instances[i] == null)
               return null;
         }
      }
      return instances;
   }

   Object[] clone(Object[] parametersToClone)
   {
      Object[] clone = (Object[]) Array.newInstance(parametersToClone.getClass().getComponentType(), parametersToClone.length);

      for (int i = 0; i < parametersToClone.length; i++)
      {
         Class<? extends Object> parameterType = parametersToClone[i].getClass();

         if (parameterType.isPrimitive() || parametersToClone[i] instanceof Number || parametersToClone[i] instanceof Boolean)
         {
            clone[i] = parametersToClone[i];
         }
         else if (DMatrix.class.isAssignableFrom(parameterType))
         {
            clone[i] = new DMatrixRMaj((DMatrix) parametersToClone[i]);
         }
         else if (float[].class.equals(parameterType))
         {
            float[] arrayToClone = (float[]) parametersToClone[i];
            clone[i] = new float[arrayToClone.length];
            System.arraycopy(arrayToClone, 0, clone[i], 0, arrayToClone.length);
         }
         else if (double[].class.equals(parameterType))
         {
            double[] arrayToClone = (double[]) parametersToClone[i];
            clone[i] = new double[arrayToClone.length];
            System.arraycopy(arrayToClone, 0, clone[i], 0, arrayToClone.length);
         }
         else if (int[].class.equals(parameterType))
         {
            int[] arrayToClone = (int[]) parametersToClone[i];
            clone[i] = new int[arrayToClone.length];
            System.arraycopy(arrayToClone, 0, clone[i], 0, arrayToClone.length);
         }
         else if (parameterType.isArray())
         {
            clone[i] = clone((Object[]) parametersToClone[i]);
         }
         else if (parameterType.isAnonymousClass())
         {
            clone[i] = parametersToClone[i];
         }
         else
         {
            try
            {
               ReferenceFrame frame = ReferenceFrame.getWorldFrame();
               if (parametersToClone[i] instanceof ReferenceFrameHolder)
                  frame = ((ReferenceFrameHolder) parametersToClone[i]).getReferenceFrame();

               clone[i] = next(new Random(), frame, parameterType);
               Method setter = parameterType.getMethod("set", parameterType);
               setter.invoke(clone[i], parametersToClone[i]);
            }
            catch (NoSuchMethodException | SecurityException | IllegalAccessException | IllegalArgumentException e)
            {
               System.err.println("Unhandled type: " + parameterType.getSimpleName());
               return null;
            }
            catch (InvocationTargetException e)
            {
               System.err.println("Unhandled type: " + parameterType.getSimpleName());
               e.getTargetException().printStackTrace();
               return null;
            }
         }
      }

      return clone;
   }

   private Object nextFramelessType(Random random, Class<?> type)
   {
      List<RandomFramelessTypeBuilder> matchingBuilders = framelessTypeBuilders.entrySet().stream().filter(entry -> type.isAssignableFrom(entry.getKey()))
                                                                               .map(Entry::getValue).collect(Collectors.toList());

      if (matchingBuilders.isEmpty())
         return null;
      else
         return matchingBuilders.get(random.nextInt(matchingBuilders.size())).newInstance(random);
   }

   private Object nextFrameType(Random random, Class<?> type, ReferenceFrame referenceFrame)
   {
      List<RandomFrameTypeBuilder> matchingBuilders = frameTypeBuilders.entrySet().stream().filter(entry -> type.isAssignableFrom(entry.getKey()))
                                                                       .map(Entry::getValue).collect(Collectors.toList());

      if (matchingBuilders.isEmpty())
         return null;
      else
         return matchingBuilders.get(random.nextInt(matchingBuilders.size())).newInstance(random, referenceFrame);
   }

   private static float[] nextFloatArray(Random random)
   {
      float[] next = new float[20];
      for (int i = 0; i < next.length; i++)
         next[i] = random.nextFloat();
      return next;
   }

   private static boolean[] nextBooleanArray(Random random)
   {
      boolean[] next = new boolean[20];
      for (int i = 0; i < next.length; i++)
         next[i] = random.nextBoolean();
      return next;
   }

   private static char[] nextCharArray(Random random)
   {
      char[] next = new char[20];
      for (int i = 0; i < next.length; i++)
         next[i] = (char) random.nextInt();
      return next;
   }
}
