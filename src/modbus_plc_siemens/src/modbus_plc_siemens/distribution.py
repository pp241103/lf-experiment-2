import math as m

colors = {0: ' \033[43m   \033[0m',
          1: ' \033[44m   \033[0m',
          2: ' \033[42m   \033[0m',
          3: ' \033[45m   \033[0m'}


# ===============================================================================================================

 
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


def get_queue(shares):
    """Transform category sizes to block queue optimally
    catsizes = [cs1, cs2, cs3, cs4] - blocks numbers of each color category
    """

    global shares
    if not shares:
        return 0

    # Defining catsizes matching the MSE-limit
    amount = 1  # starting amount
    lim = 0.03  # MSE-limit
    while True:
        error = 0
        catsizes = getsizes(shares, amount)
        for cs, w in zip(catsizes, shares):
            error += (cs / amount - w) ** 2
        error = m.sqrt(error / 4)
        if error > lim:
            amount += 1
        else:
            break

    # ======================================================================
    # Evenly distributing algorithm of block queue using dimensional method
    # (catsizes = (cs1; cs2; cs3; cs4) - 4D-vector)

    fullvec = sum([cs * cs for cs in catsizes])
    passedvec = 0

    point = [0] * 4
    delta = [0] * 4

    queue = []

    for _ in range(sum(catsizes)):
        # Defining the minimal delta for each point (???)
        for coord in range(4):
            delta[coord] = (2 * point[coord] + 1) * fullvec - (2 * passedvec + catsizes[coord]) * catsizes[coord]

        bestcoord = delta.index(min(delta))
        passedvec += catsizes[bestcoord]
        point[bestcoord] += 1

        queue.append(bestcoord)
    # ======================================================================
    return queue


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
            queue = distribute(shares)
            
            for d, ln in zip(queue, range(len(queue))):
                blocks = blocks + colors[d]
                if (ln + 1) % 20 == 0 and ln + 1 != length:
                    blocks = blocks + '\n\n'

            print(f'{blocks}\n')
            print(f'\033[33mYellow blocks: {queue.count(0)} '
                  f'(block share: {catsizes[0] / length:.5g}; wallet share: {wallets[0]})\033[0m')
            print(f'\033[34mBlue blocks: {queue.count(1)} '
                  f'(block share: {catsizes[1] / length:.5g}; wallet share: {wallets[1]})\033[0m')
            print(f'\033[32mGreen blocks: {queue.count(2)} '
                  f'(block share: {catsizes[2] / length:.5g}; wallet share: {wallets[2]})\033[0m')
            print(f'\033[35mPurple blocks: {queue.count(3)} '
                  f'(block share: {catsizes[3] / length:.5g}; wallet share: {wallets[3]}\033[0m)')
            print('=' * 80 + '\n')
            
        except:
            print('- Error: incorrect entered data!\n' + '=' * 80 + '\n')
            continue
    