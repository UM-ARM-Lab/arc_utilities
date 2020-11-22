import traceback


def conditional_try(conditional, function, **kwargs):
    if conditional:
        try:
            return function(**kwargs)
        except Exception:
            print("Caught an exception!")
            traceback.print_exc()
            print("End of caught exception.")
    else:
        return function(**kwargs)