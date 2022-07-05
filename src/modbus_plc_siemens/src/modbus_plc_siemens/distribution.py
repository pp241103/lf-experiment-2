import math as m

colors = {0: ' \033[43m   \033[0m',
          1: ' \033[44m   \033[0m',
          2: ' \033[42m   \033[0m',
          3: ' \033[45m   \033[0m'}


# =======================================================================================================

def get_sizes(shares, amount):
    """Transform wallet shares to category sizes optimally
    catsizes = [cs1, cs2, cs3, cs4] - blocks numbers of each color category
    :param shares: list of wallet shares
    :type shares: list in range (4)
    :param amount: total amount of blocks
    :type amount: int
    :return: list of category sizes
    :rtype: list in range (4)
    """

    catsizes = [w * amount for w in shares]

    for catsize in catsizes:
        # if any block catsize is non-integer...
        if catsize - int(catsize) > 0:
            # ==============================================================
            # Round all shares optimally (0.5, +1, -1)
            trig = True
            for k, cs in enumerate(catsizes):
                if cs - int(cs) == 0.5:
                    if trig:
                        catsizes[k] = int(cs + 0.5)
                        trig = False
                    else:
                        catsizes[k] = int(cs - 0.5)
                        trig = True
                else:
                    catsizes[k] = round(cs)
            # ==============================================================
            if amount - sum(catsizes) == 1:
                maxcat = max([cs - int(cs) for cs in catsizes])
                for k, cs in enumerate(catsizes):
                    if cs - int(cs) == maxcat:
                        catsizes[k] += 1
                        break
            elif sum(catsizes) - amount == 1:
                mincat = min([cs - int(cs) for cs in catsizes])
                for k, cs in reversed(list(enumerate(catsizes))):
                    if cs - int(cs) == mincat:
                        catsizes[k] -= 1
                        break
            # ==============================================================
            return catsizes
    else:
        return [int(cs) for cs in catsizes]


def get_queue(shares, length=None):
    """Transform category sizes to block queue optimally
    catsizes = [cs1, cs2, cs3, cs4] - blocks numbers of each color category
    """
    
    if length:  # Defining catsizes matching fixed amount
        catsizes = get_sizes(shares, length)
    else:  # Defining catsizes matching the MSE-limit
        amount, lim = 1, 0.03  # starting amount / MSE-limit
        while True:
            catsizes = get_sizes(shares, amount)
            error = m.sqrt(sum([(cs / amount - w) ** 2 for cs, w in zip(catsizes, shares)]) / 4)
            if error > lim:
                amount += 1
            else:
                break

    # ======================================================================
    # Evenly distributing algorithm of block queue using dimensional method
    # (catsizes = (cs1; cs2; cs3; cs4) - 4D-vector)

    norm_cs = m.sqrt(sum([cs * cs for cs in catsizes]))
    point, delta, queue = [0] * 4, [0.] * 4, []

    for _ in range(sum(catsizes)):
        # Defining the minimal delta for each point
        for coord in range(4):
            newpoint = point.copy()
            newpoint[coord] += 1
            
            p_dot_cs = sum([p * cs for p, cs in zip(newpoint, catsizes)])
            norm_p = m.sqrt(sum([p * p for p in newpoint]))
            
            delta[coord] = m.sqrt(abs(norm_p ** 2 - (p_dot_cs / norm_cs) ** 2))
        
        # Moving to the point with minimal delta
        bestcoord = delta.index(min(delta))
        point[bestcoord] += 1

        queue.append(bestcoord)
    # ======================================================================
    return queue

# =======================================================================================================

if __name__ == "__main__":

    while True:
        try:
            if input('Run the algorithim? (0 or another key): ') == '0':
                break
            yellow = eval(input('\033[33mEnter yellow share: \033[0m'))
            blue = eval(input('\033[34mEnter blue share: \033[0m'))
            green = eval(input('\033[32mEnter green share: \033[0m'))
            purple = eval(input('\033[35mEnter purple share: \033[0m'))

            shares = [yellow, blue, green, purple]
            assert all(s >= 0 for s in shares) and sum(shares) == 1
            length = int(input('Enter your length or zero: '))
            assert length >= 0
            
            queue = get_queue(shares, length)

            blocks, length = None, len(queue)
            for d, ln in zip(queue, range(length)):
                blocks = blocks + colors[d]
                if (ln + 1) % 20 == 0 and ln + 1 != length:
                    blocks = blocks + '\n\n'

            print(f'{blocks}\n')
            print(f'\033[33mYellow blocks: {queue.count(0)} '
                  f'(block share: {queue.count(0) / length:.5g}; wallet share: {shares[0]})\033[0m')
            print(f'\033[34mBlue blocks: {queue.count(1)} '
                  f'(block share: {queue.count(1) / length:.5g}; wallet share: {shares[1]})\033[0m')
            print(f'\033[32mGreen blocks: {queue.count(2)} '
                  f'(block share: {queue.count(2) / length:.5g}; wallet share: {shares[2]})\033[0m')
            print(f'\033[35mPurple blocks: {queue.count(3)} '
                  f'(block share: {queue.count(3) / length:.5g}; wallet share: {shares[3]}\033[0m)')
            print('=' * 80 + '\n')

        except:
            print('- Error: incorrect entered data!\n' + '=' * 80 + '\n')
            continue

# =======================================================================================================
























































